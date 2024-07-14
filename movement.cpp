#include "includes.h"

Movement g_movement{ };;

void Movement::JumpRelated( ) {
	if( g_cl.m_local->m_MoveType( ) == MOVETYPE_NOCLIP )
		return;

	if( ( g_cl.m_cmd->m_buttons & IN_JUMP ) && !( g_cl.m_flags & FL_ONGROUND ) ) {
		// bhop.
		if( g_menu.main.misc.bhop.get( ) )
			g_cl.m_cmd->m_buttons &= ~IN_JUMP;

		// duck jump ( crate jump ).
		if( g_menu.main.misc.airduck.get( ) )
			g_cl.m_cmd->m_buttons |= IN_DUCK;
	}
}

void Movement::Strafe() {
	vec3_t velocity;
	float  delta, abs_delta, velocity_delta, correct;

	// the strafe in air removal (real).
	if (g_cl.m_flags & FL_ONGROUND || g_input.GetKeyState(g_menu.main.misc.instant_stop_in_air.get()) || g_aimbot.m_stop_air)
		return;

	// don't strafe while noclipping or on ladders..
	if (g_cl.m_local->m_MoveType() == MOVETYPE_NOCLIP || g_cl.m_local->m_MoveType() == MOVETYPE_LADDER)
		return;

	// get networked velocity ( maybe absvelocity better here? ).
	// meh, should be predicted anyway? ill see.
	velocity = g_cl.m_local->m_vecAbsVelocity();

	// get the velocity len2d ( speed ).
	m_speed = velocity.length_2d();

	// compute the ideal strafe angle for our velocity.
	m_ideal = (m_speed > 0.f) ? math::rad_to_deg(std::asin(15.f / m_speed)) : 90.f;
	m_ideal2 = (m_speed > 0.f) ? math::rad_to_deg(std::asin(30.f / m_speed)) : 90.f;

	// some additional sanity.
	math::clamp(m_ideal, 0.f, 90.f);
	math::clamp(m_ideal2, 0.f, 90.f);

	// save entity bounds ( used much in circle-strafer ).
	m_mins = g_cl.m_local->m_vecMins();
	m_maxs = g_cl.m_local->m_vecMaxs();

	// save our origin
	m_origin = g_cl.m_local->m_vecOrigin();

	// disable strafing while pressing shift.
	if ((g_cl.m_buttons & IN_SPEED) || (g_cl.m_flags & FL_ONGROUND))
		return;

	// for changing direction.
	// we want to change strafe direction every call.
	m_switch_value *= -1.f;

	// for allign strafer.
	++m_strafe_index;

	if (g_cl.m_pressing_move && g_menu.main.misc.autostrafe.get()) {
		// took this idea from stacker, thank u !!!!
		enum EDirections {
			FORWARDS = 0,
			BACKWARDS = 180,
			LEFT = 90,
			RIGHT = -90,
			BACK_LEFT = 135,
			BACK_RIGHT = -135
		};

		float wish_dir{ };

		// get our key presses.
		bool holding_w = g_cl.m_buttons & IN_FORWARD;
		bool holding_a = g_cl.m_buttons & IN_MOVELEFT;
		bool holding_s = g_cl.m_buttons & IN_BACK;
		bool holding_d = g_cl.m_buttons & IN_MOVERIGHT;

		// move in the appropriate direction.
		if (holding_w) {
			//    forward left
			if (holding_a) {
				wish_dir += (EDirections::LEFT / 2);
			}
			//    forward right
			else if (holding_d) {
				wish_dir += (EDirections::RIGHT / 2);
			}
			//    forward
			else {
				wish_dir += EDirections::FORWARDS;
			}
		}
		else if (holding_s) {
			//    back left
			if (holding_a) {
				wish_dir += EDirections::BACK_LEFT;
			}
			//    back right
			else if (holding_d) {
				wish_dir += EDirections::BACK_RIGHT;
			}
			//    back
			else {
				wish_dir += EDirections::BACKWARDS;
			}

			g_cl.m_cmd->m_forward_move = 0;
		}
		else if (holding_a) {
			//    left
			wish_dir += EDirections::LEFT;
		}
		else if (holding_d) {
			//    right
			wish_dir += EDirections::RIGHT;
		}

		g_cl.m_strafe_angles.y += math::NormalizeYaw(wish_dir);
	}

	// cancel out any forwardmove values.
	g_cl.m_cmd->m_forward_move = 0.f;

	if (!g_menu.main.misc.autostrafe.get())
		return;

	// get our viewangle change.
	delta = math::NormalizedAngle(g_cl.m_strafe_angles.y - m_old_yaw);

	// convert to absolute change.
	abs_delta = std::abs(delta);

	// save old yaw for next call.
	m_circle_yaw = m_old_yaw = g_cl.m_strafe_angles.y;

	// set strafe direction based on mouse direction change.
	if (delta > 0.f)
		g_cl.m_cmd->m_side_move = -450.f;

	else if (delta < 0.f)
		g_cl.m_cmd->m_side_move = 450.f;

	// we can accelerate more, because we strafed less then needed
	// or we got of track and need to be retracked.
	if (abs_delta <= m_ideal || abs_delta >= 30.f) {
		// compute angle of the direction we are traveling in.
		ang_t velocity_angle;
		math::VectorAngles(velocity, velocity_angle);

		// get the delta between our direction and where we are looking at.
		velocity_delta = math::NormalizeYaw(g_cl.m_strafe_angles.y - velocity_angle.y);

		// correct our strafe amongst the path of a circle.
		correct = m_ideal;

		if (velocity_delta <= correct || m_speed <= 15.f) {
			// not moving mouse, switch strafe every tick.
			if (-correct <= velocity_delta || m_speed <= 15.f) {
				g_cl.m_strafe_angles.y += (m_ideal * m_switch_value);
				g_cl.m_cmd->m_side_move = 450.f * m_switch_value;
			}

			else {
				g_cl.m_strafe_angles.y = velocity_angle.y - correct;
				g_cl.m_cmd->m_side_move = 450.f;
			}
		}

		else {
			g_cl.m_strafe_angles.y = velocity_angle.y + correct;
			g_cl.m_cmd->m_side_move = -450.f;
		}
	}
}

void Movement::DoPrespeed( ) {
	float   mod, min, max, step, strafe, time, angle;
	vec3_t  plane;

	// min and max values are based on 128 ticks.
	mod = g_csgo.m_globals->m_interval * 128.f;

	// scale min and max based on tickrate.
	min = 2.25f * mod;
	max = 5.f * mod;

	// compute ideal strafe angle for moving in a circle.
	strafe = m_ideal * 2.f;

	// clamp ideal strafe circle value to min and max step.
	math::clamp( strafe, min, max );

	// calculate time.
	time = 320.f / m_speed;

	// clamp time.
	math::clamp( time, 0.35f, 1.f );

	// init step.
	step = strafe;

	while( true ) {
		// if we will not collide with an object or we wont accelerate from such a big step anymore then stop.
		if( !WillCollide( time, step ) || max <= step )
			break;

		// if we will collide with an object with the current strafe step then increment step to prevent a collision.
		step += 0.2f;
	}

	if( step > max ) {
		// reset step.
		step = strafe;

		while( true ) {
			// if we will not collide with an object or we wont accelerate from such a big step anymore then stop.
			if( !WillCollide( time, step ) || step <= -min )
				break;

			// if we will collide with an object with the current strafe step decrement step to prevent a collision.
			step -= 0.2f;
		}

		if( step < -min ) {
			if( GetClosestPlane( plane ) ) {
				// grab the closest object normal
				// compute the angle of the normal
				// and push us away from the object.
				angle = math::rad_to_deg( std::atan2( plane.y, plane.x ) );
				step = -math::NormalizedAngle( m_circle_yaw - angle ) * 0.1f;
			}
		}

		else
			step -= 0.2f;
	}

	else
		step += 0.2f;

	// add the computed step to the steps of the previous circle iterations.
	m_circle_yaw = math::NormalizedAngle( m_circle_yaw + step );

	// apply data to usercmd.
	g_cl.m_cmd->m_view_angles.y = m_circle_yaw;
	g_cl.m_cmd->m_side_move = ( step >= 0.f ) ? -450.f : 450.f;
}

bool Movement::GetClosestPlane( vec3_t &plane ) {
	CGameTrace            trace;
	CTraceFilterWorldOnly filter;
	vec3_t                start{ m_origin };
	float                 smallest{ 1.f };
	const float		      dist{ 75.f };

	// trace around us in a circle
	for( float step{ }; step <= math::pi_2; step += ( math::pi / 10.f ) ) {
		// extend endpoint x units.
		vec3_t end = start;
		end.x += std::cos( step ) * dist;
		end.y += std::sin( step ) * dist;

		g_csgo.m_engine_trace->TraceRay( Ray( start, end, m_mins, m_maxs ), CONTENTS_SOLID, &filter, &trace );

		// we found an object closer, then the previouly found object.
		if( trace.m_fraction < smallest ) {
			// save the normal of the object.
			plane = trace.m_plane.m_normal;
			smallest = trace.m_fraction;
		}
	}

	// did we find any valid object?
	return smallest != 1.f && plane.z < 0.1f;
}

bool Movement::WillCollide( float time, float change ) {
	struct PredictionData_t {
		vec3_t start;
		vec3_t end;
		vec3_t velocity;
		float  direction;
		bool   ground;
		float  predicted;
	};

	PredictionData_t      data;
	CGameTrace            trace;
	CTraceFilterWorldOnly filter;

	// set base data.
	data.ground = g_cl.m_flags & FL_ONGROUND;
	data.start = m_origin;
	data.end = m_origin;
	data.velocity = g_cl.m_local->m_vecVelocity( );
	data.direction = math::rad_to_deg( std::atan2( data.velocity.y, data.velocity.x ) );

	for( data.predicted = 0.f; data.predicted < time; data.predicted += g_csgo.m_globals->m_interval ) {
		// predict movement direction by adding the direction change.
		// make sure to normalize it, in case we go over the -180/180 turning point.
		data.direction = math::NormalizedAngle( data.direction + change );

		// pythagoras.
		float hyp = data.velocity.length_2d( );

		// adjust velocity for new direction.
		data.velocity.x = std::cos( math::deg_to_rad( data.direction ) ) * hyp;
		data.velocity.y = std::sin( math::deg_to_rad( data.direction ) ) * hyp;

		// assume we bhop, set upwards impulse.
		if( data.ground )
			data.velocity.z = g_csgo.sv_jump_impulse->GetFloat( );

		else
			data.velocity.z -= g_csgo.sv_gravity->GetFloat( ) * g_csgo.m_globals->m_interval;

		// we adjusted the velocity for our new direction.
		// see if we can move in this direction, predict our new origin if we were to travel at this velocity.
		data.end += ( data.velocity * g_csgo.m_globals->m_interval );

		// trace
		g_csgo.m_engine_trace->TraceRay( Ray( data.start, data.end, m_mins, m_maxs ), MASK_PLAYERSOLID, &filter, &trace );

		// check if we hit any objects.
		if( trace.m_fraction != 1.f && trace.m_plane.m_normal.z <= 0.9f )
			return true;
		if( trace.m_startsolid || trace.m_allsolid )
			return true;

		// adjust start and end point.
		data.start = data.end = trace.m_endpos;

		// move endpoint 2 units down, and re-trace.
		// do this to check if we are on th floor.
		g_csgo.m_engine_trace->TraceRay( Ray( data.start, data.end - vec3_t{ 0.f, 0.f, 2.f }, m_mins, m_maxs ), MASK_PLAYERSOLID, &filter, &trace );

		// see if we moved the player into the ground for the next iteration.
		data.ground = trace.hit( ) && trace.m_plane.m_normal.z > 0.7f;
	}

	// the entire loop has ran
	// we did not hit shit.
	return false;
}

void Movement::FixMove( CUserCmd *cmd, const ang_t &wish_angles ) {
	vec3_t view_fwd, view_right, view_up, cmd_fwd, cmd_right, cmd_up;

	math::AngleVectors(wish_angles, &view_fwd, &view_right, &view_up);
	math::AngleVectors(cmd->m_view_angles, &cmd_fwd, &cmd_right, &cmd_up);

	const auto v8 = sqrtf((view_fwd.x * view_fwd.x) + (view_fwd.y * view_fwd.y));
	const auto v10 = sqrtf((view_right.x * view_right.x) + (view_right.y * view_right.y));
	const auto v12 = sqrtf(view_up.z * view_up.z);

	const vec3_t norm_view_fwd((1.f / v8) * view_fwd.x, (1.f / v8) * view_fwd.y, 0.f);
	const vec3_t norm_view_right((1.f / v10) * view_right.x, (1.f / v10) * view_right.y, 0.f);
	const vec3_t norm_view_up(0.f, 0.f, (1.f / v12) * view_up.z);

	const auto v14 = sqrtf((cmd_fwd.x * cmd_fwd.x) + (cmd_fwd.y * cmd_fwd.y));
	const auto v16 = sqrtf((cmd_right.x * cmd_right.x) + (cmd_right.y * cmd_right.y));
	const auto v18 = sqrtf(cmd_up.z * cmd_up.z);

	const vec3_t norm_cmd_fwd((1.f / v14) * cmd_fwd.x, (1.f / v14) * cmd_fwd.y, 0.f);
	const vec3_t norm_cmd_right((1.f / v16) * cmd_right.x, (1.f / v16) * cmd_right.y, 0.f);
	const vec3_t norm_cmd_up(0.f, 0.f, (1.f / v18) * cmd_up.z);

	const auto v22 = norm_view_fwd.x * cmd->m_forward_move;
	const auto v26 = norm_view_fwd.y * cmd->m_forward_move;
	const auto v28 = norm_view_fwd.z * cmd->m_forward_move;
	const auto v24 = norm_view_right.x * cmd->m_side_move;
	const auto v23 = norm_view_right.y * cmd->m_side_move;
	const auto v25 = norm_view_right.z * cmd->m_side_move;
	const auto v30 = norm_view_up.x * cmd->m_up_move;
	const auto v27 = norm_view_up.z * cmd->m_up_move;
	const auto v29 = norm_view_up.y * cmd->m_up_move;

	cmd->m_forward_move = ((((norm_cmd_fwd.x * v24) + (norm_cmd_fwd.y * v23)) + (norm_cmd_fwd.z * v25))
		+ (((norm_cmd_fwd.x * v22) + (norm_cmd_fwd.y * v26)) + (norm_cmd_fwd.z * v28)))
		+ (((norm_cmd_fwd.y * v30) + (norm_cmd_fwd.x * v29)) + (norm_cmd_fwd.z * v27));

	cmd->m_side_move = ((((norm_cmd_right.x * v24) + (norm_cmd_right.y * v23)) + (norm_cmd_right.z * v25))
		+ (((norm_cmd_right.x * v22) + (norm_cmd_right.y * v26)) + (norm_cmd_right.z * v28)))
		+ (((norm_cmd_right.x * v29) + (norm_cmd_right.y * v30)) + (norm_cmd_right.z * v27));

	cmd->m_up_move = ((((norm_cmd_up.x * v23) + (norm_cmd_up.y * v24)) + (norm_cmd_up.z * v25))
		+ (((norm_cmd_up.x * v26) + (norm_cmd_up.y * v22)) + (norm_cmd_up.z * v28)))
		+ (((norm_cmd_up.x * v30) + (norm_cmd_up.y * v29)) + (norm_cmd_up.z * v27));

	if (g_cl.m_local->m_MoveType() != MOVETYPE_LADDER)
		cmd->m_buttons &= ~(IN_FORWARD | IN_BACK | IN_MOVERIGHT | IN_MOVELEFT);
}

void Movement::AutoPeek(CUserCmd* cmd, float wish_yaw) {
	// set to invert if we press the button.
	if (g_input.GetKeyState(g_menu.main.aimbot.quickpeekassist.get())) {
		if (start_position.IsZero()) {
			start_position = g_cl.m_local->GetAbsOrigin();

			if (!(g_cl.m_flags & FL_ONGROUND)) {
				CTraceFilterWorldOnly filter;
				CGameTrace trace;

				g_csgo.m_engine_trace->TraceRay(Ray(start_position, start_position - vec3_t(0.0f, 0.0f, 1000.0f)), MASK_SOLID, &filter, &trace);

				if (trace.m_fraction < 1.0f)
					start_position = trace.m_endpos + vec3_t(0.0f, 0.0f, 2.0f);
			}
		}
		else {
			bool revolver_shoot = g_cl.m_weapon_id == REVOLVER && !g_cl.m_revolver_fire && (cmd->m_buttons & IN_ATTACK || cmd->m_buttons & IN_ATTACK2);

			if (g_cl.m_old_shot)
				fired_shot = true;

			if (fired_shot) {
				vec3_t current_position = g_cl.m_local->GetAbsOrigin();
				vec3_t difference = current_position - start_position;

				if (difference.length_2d() > 5.0f) {
					vec3_t velocity = vec3_t(difference.x * cos(wish_yaw / 180.0f * math::pi) + difference.y * sin(wish_yaw / 180.0f * math::pi), difference.y * cos(wish_yaw / 180.0f * math::pi) - difference.x * sin(wish_yaw / 180.0f * math::pi), difference.z);

					if (difference.length_2d() < 50.0f) {
						cmd->m_forward_move = -velocity.x * 20.0f;
						cmd->m_side_move = velocity.y * 20.0f;
					}
					else if (difference.length_2d() < 100.0f) {
						cmd->m_forward_move = -velocity.x * 10.0f;
						cmd->m_side_move = velocity.y * 10.0f;
					}
					else if (difference.length_2d() < 150.0f) {
						cmd->m_forward_move = -velocity.x * 5.0f;
						cmd->m_side_move = velocity.y * 5.0f;
					}
					else if (difference.length_2d() < 250.0f) {
						cmd->m_forward_move = -velocity.x * 2.0f;
						cmd->m_side_move = velocity.y * 2.0f;
					}
					else {
						cmd->m_forward_move = -velocity.x * 1.0f;
						cmd->m_side_move = velocity.y * 1.0f;
					}
				}
				else {
					fired_shot = false;
					start_position.clear();
				}
			}
		}
	}
	else {
		fired_shot = false;
		start_position.clear();

		// memory leak
		return;
	}
}


void Movement::accelerate(const CUserCmd& cmd, const vec3_t& wishdir, const float wishspeed, vec3_t& velocity, float acceleration) {
	const auto cur_speed = velocity.dot(wishdir);

	static auto sv_accelerate_use_weapon_speed = g_csgo.m_cvar->FindVar(HASH("sv_accelerate_use_weapon_speed"));

	const auto add_speed = wishspeed - cur_speed;
	if (add_speed <= 0.f)
		return;

	const auto v57 = std::max(cur_speed, 0.f);

	const auto ducking =
		cmd.m_buttons & IN_DUCK;

	auto v20 = true;
	if (ducking
		|| !(cmd.m_buttons & IN_SPEED))
		v20 = false;

	auto finalwishspeed = std::max(wishspeed, 250.f);
	auto abs_finalwishspeed = finalwishspeed;

	const auto weapon = g_cl.m_local->GetActiveWeapon();

	bool slow_down_to_fast_nigga{};

	if (weapon
		&& sv_accelerate_use_weapon_speed->GetInt()) {
		const auto item_index = static_cast<std::uint16_t>(weapon->m_iItemDefinitionIndex());
		if (weapon->m_zoomLevel() > 0
			&& (item_index == 11 || item_index == 38 || item_index == 9 || item_index == 8 || item_index == 39 || item_index == 40))
			slow_down_to_fast_nigga = (g_movement.m_max_weapon_speed * 0.52f) < 110.f;

		const auto modifier = std::min(1.f, g_movement.m_max_weapon_speed / 250.f);

		abs_finalwishspeed *= modifier;

		if ((!ducking && !v20)
			|| slow_down_to_fast_nigga)
			finalwishspeed *= modifier;
	}

	if (ducking) {
		if (!slow_down_to_fast_nigga)
			finalwishspeed *= 0.34f;

		abs_finalwishspeed *= 0.34f;
	}

	if (v20) {
		if (!slow_down_to_fast_nigga)
			finalwishspeed *= 0.52f;

		abs_finalwishspeed *= 0.52f;

		const auto abs_finalwishspeed_minus5 = abs_finalwishspeed - 5.f;
		if (v57 < abs_finalwishspeed_minus5) {
			const auto v30 =
				std::max(v57 - abs_finalwishspeed_minus5, 0.f)
				/ std::max(abs_finalwishspeed - abs_finalwishspeed_minus5, 0.f);

			const auto v27 = 1.f - v30;
			if (v27 >= 0.f)
				acceleration = std::min(v27, 1.f) * acceleration;
			else
				acceleration = 0.f;
		}
	}

	const auto v33 = std::min(
		add_speed,
		((g_csgo.m_globals->m_interval * acceleration) * finalwishspeed)
		* g_cl.m_local->m_surfaceFriction()
	);

	velocity += wishdir * v33;

	const auto len = velocity.length();
	if (len
		&& len > g_movement.m_max_weapon_speed)
		velocity *= g_movement.m_max_weapon_speed / len;

}

void Movement::walk_move(const CUserCmd& cmd, vec3_t& move, vec3_t& fwd, vec3_t& right, vec3_t& velocity) {
	if (fwd.z != 0.f)
		fwd.normalize();

	if (right.z != 0.f)
		right.normalize();

	vec3_t wishvel{
		fwd.x * move.x + right.x * move.y,
		fwd.y * move.x + right.y * move.y,
		0.f
	};
	static auto sv_accelerate = g_csgo.m_cvar->FindVar(HASH("sv_accelerate"));
	auto wishdir = wishvel;

	auto wishspeed = wishdir.normalize();
	if (wishspeed
		&& wishspeed > g_movement.m_max_player_speed) {
		wishvel *= g_movement.m_max_player_speed / wishspeed;

		wishspeed = g_movement.m_max_player_speed;
	}

	velocity.z = 0.f;
	accelerate(cmd, wishdir, wishspeed, velocity, sv_accelerate->GetFloat());
	velocity.z = 0.f;

	const auto speed_sqr = velocity.length_sqr();
	if (speed_sqr > (g_movement.m_max_player_speed * g_movement.m_max_player_speed))
		velocity *= g_movement.m_max_player_speed / std::sqrt(speed_sqr);

	if (velocity.length() < 1.f)
		velocity = {};
}

void Movement::full_walk_move(const CUserCmd cmd, vec3_t move, vec3_t fwd, vec3_t right, vec3_t velocity) {
	static auto sv_maxvelocity = g_csgo.m_cvar->FindVar(HASH("sv_maxvelocity"));
	static auto sv_friction = g_csgo.m_cvar->FindVar(HASH("sv_friction"));

	if (reinterpret_cast <ulong_t>(g_cl.m_local->GetGroundEntity())) {
		velocity.z = 0.f;

		const auto speed = velocity.length();
		if (speed >= 0.1f) {
			const auto friction = sv_friction->GetFloat() * g_cl.m_local->m_surfaceFriction();
			const auto sv_stopspeed = sv_friction->GetFloat();
			const auto control = speed < sv_stopspeed ? sv_stopspeed : speed;

			const auto new_speed = std::max(0.f, speed - ((control * friction) * g_csgo.m_globals->m_interval));
			if (speed != new_speed)
				velocity *= new_speed / speed;
		}

		walk_move(cmd, move, fwd, right, velocity);

		velocity.z = 0.f;
	}

	const auto sv_maxvelocity_ = sv_maxvelocity->GetFloat();
	for (std::size_t i{}; i < 3u; ++i) {
		auto& element = velocity.at(i);

		if (element > sv_maxvelocity_)
			element = sv_maxvelocity_;
		else if (element < -sv_maxvelocity_)
			element = -sv_maxvelocity_;
	}
}

void Movement::modify_move(CUserCmd cmd, vec3_t velocity) {
	vec3_t fwd{}, right{};

	math::AngleVectors(cmd.m_view_angles, &fwd, &right, nullptr);

	const auto speed_sqr = vec3_t(cmd.m_side_move, cmd.m_forward_move, cmd.m_up_move).length_sqr();
	if (speed_sqr > (g_movement.m_max_player_speed * g_movement.m_max_player_speed))
		vec3_t(cmd.m_side_move, cmd.m_forward_move, cmd.m_up_move) *= g_movement.m_max_player_speed / std::sqrt(speed_sqr);

	full_walk_move(cmd, vec3_t(cmd.m_side_move, cmd.m_forward_move, cmd.m_up_move), fwd, right, velocity);
}

void Movement::predict_move(const CUserCmd cmd, vec3_t velocity) {
	vec3_t fwd{}, right{};

	math::AngleVectors(cmd.m_view_angles, &fwd, &right, nullptr);

	auto move = vec3_t(cmd.m_side_move, cmd.m_forward_move, cmd.m_up_move);

	const auto speed_sqr = vec3_t(cmd.m_side_move, cmd.m_forward_move, cmd.m_up_move).length_sqr();
	if (speed_sqr > (g_movement.m_max_player_speed * g_movement.m_max_player_speed))
		move *= g_movement.m_max_player_speed / std::sqrt(speed_sqr);

	full_walk_move(cmd, move, fwd, right, velocity);
}

void Movement::ClampMovementSpeed(float speed) {

	CUserCmd* cmd = g_cl.m_cmd;

	cmd->m_buttons &= ~IN_SPEED;

	float movement_speed = std::sqrtf(cmd->m_forward_move * cmd->m_forward_move + cmd->m_side_move * cmd->m_side_move);
	if (movement_speed > 28.f) {
		if (movement_speed > speed) {
			float mov_speed = g_cl.m_local->m_vecVelocity().length_2d();

			float forward_ratio = cmd->m_forward_move / movement_speed;
			float side_ratio = cmd->m_side_move / movement_speed;

			cmd->m_forward_move = forward_ratio * std::min(movement_speed, speed);
			cmd->m_side_move = side_ratio * std::min(movement_speed, speed);
		}
	}
}

void RotateMovement(CUserCmd* pCmd, ang_t& angOldViewPoint) {
	// get max speed limits by convars
	const float flMaxForwardSpeed = 450.f;
	const float flMaxSideSpeed = 450.f;
	const float flMaxUpSpeed = 320.f;

	vec3_t vecForward = { }, vecRight = { }, vecUp = { };
	math::AngleVectors(angOldViewPoint, &vecForward, &vecRight, &vecUp);

	// we don't attempt on forward/right roll, and on up pitch/yaw
	vecForward.z = vecRight.z = vecUp.x = vecUp.y = 0.f;

	vecForward.normalize();
	vecRight.normalize();
	vecUp.normalize();

	vec3_t vecOldForward = { }, vecOldRight = { }, vecOldUp = { };
	math::AngleVectors(pCmd->m_view_angles, &vecOldForward, &vecOldRight, &vecOldUp);

	// we don't attempt on forward/right roll, and on up pitch/yaw
	vecOldForward.z = vecOldRight.z = vecOldUp.x = vecOldUp.y = 0.f;

	vecOldForward.normalize();
	vecOldRight.normalize();
	vecOldUp.normalize();

	const float flPitchForward = vecForward.x * pCmd->m_forward_move;
	const float flYawForward = vecForward.y * pCmd->m_forward_move;
	const float flPitchSide = vecRight.x * pCmd->m_side_move;
	const float flYawSide = vecRight.y * pCmd->m_side_move;
	const float flRollUp = vecUp.z * pCmd->m_up_move;

	// solve corrected movement
	const float x = vecOldForward.x * flPitchSide + vecOldForward.y * flYawSide + vecOldForward.x * flPitchForward + vecOldForward.y * flYawForward + vecOldForward.z * flRollUp;
	const float y = vecOldRight.x * flPitchSide + vecOldRight.y * flYawSide + vecOldRight.x * flPitchForward + vecOldRight.y * flYawForward + vecOldRight.z * flRollUp;
	const float z = vecOldUp.x * flYawSide + vecOldUp.y * flPitchSide + vecOldUp.x * flYawForward + vecOldUp.y * flPitchForward + vecOldUp.z * flRollUp;

	// clamp and apply corrected movement
	pCmd->m_forward_move = std::clamp(x, -flMaxForwardSpeed, flMaxForwardSpeed);
	pCmd->m_side_move = std::clamp(y, -flMaxSideSpeed, flMaxSideSpeed);
	pCmd->m_up_move = std::clamp(z, -flMaxUpSpeed, flMaxUpSpeed);
}

void Movement::NullVelocity() {
	vec3_t Velocity = g_cl.m_local->m_vecVelocity();

	ang_t direction;
	ang_t real_view = g_cl.m_cmd->m_view_angles;

	math::VectorAngles(Velocity, direction);
	g_csgo.m_engine->GetViewAngles(real_view);

	direction.y = real_view.y - direction.y;

	vec3_t forward;
	math::AngleVectors(direction, &forward);

	static ConVar* cl_forwardspeed = g_csgo.m_cvar->FindVar(HASH("cl_forwardspeed"));
	static ConVar* cl_sidespeed = g_csgo.m_cvar->FindVar(HASH("cl_sidespeed"));

	const float negative_forward_speed = -cl_forwardspeed->GetFloat();
	const float negative_side_speed = -cl_sidespeed->GetFloat();

	const vec3_t negative_forward_direction = forward * negative_forward_speed;
	const vec3_t negative_side_direction = forward * negative_side_speed;

	g_cl.m_cmd->m_forward_move = negative_forward_direction.x;
	g_cl.m_cmd->m_side_move = negative_side_direction.y;
}

void Movement::AutoStop() {
	if (!g_cl.m_weapon_info)
		return;

	if (!g_cl.m_weapon) // sanity check
		return;

	if (!g_cl.m_local || !g_cl.m_processing)
		return;
	
	if ( !g_menu.main.aimbot.quick_stop.get() || !g_aimbot.m_stop )
		return;

	Weapon* wpn = g_cl.m_local->GetActiveWeapon();

	if (!wpn)
		return;

	WeaponInfo* wpn_data = wpn->GetWpnData();


	if (!wpn_data)
		return;

	bool full_stop = !g_menu.main.aimbot.quick_stop_mode.get(0);
	float max_speed = std::floor( 0.2f * (g_cl.m_local->m_bIsScoped() ? wpn_data->m_max_player_speed_alt : wpn_data->m_max_player_speed) );

	if( full_stop || g_cl.m_weapon_id == WEAPON_ZEUS || !( g_cl.m_flags & FL_ONGROUND ) )
		max_speed = 25.f;

	if (g_cl.m_local->m_vecVelocity().length_2d() < max_speed) {

		if( full_stop )
			g_cl.m_cmd->m_forward_move = g_cl.m_cmd->m_side_move = 0.f;
		else
			ClampMovementSpeed(max_speed);
	}
	else {
		NullVelocity();
	}
}

void Movement::FakeWalk( ) {
	vec3_t velocity{ g_cl.m_local->m_vecVelocity( ) };
	int    ticks_to_stop{ }, max{ 14 };

	if( !g_input.GetKeyState( g_menu.main.misc.fakewalk.get( ) ) || !(g_cl.m_flags & FL_ONGROUND) )
		return;

	if( !g_cl.m_local->GetGroundEntity( ) )
		return;

	
	Weapon* wpn = g_cl.m_local->GetActiveWeapon();

	if (!wpn)
		return;

	WeaponInfo* wpn_data = wpn->GetWpnData();


	if (!wpn_data)
		return;
	
	// reference:
	// https://github.com/ValveSoftware/source-sdk-2013/blob/master/mp/src/game/shared/gamemovement.cpp#L1612

	// calculate friction.
	float friction = g_csgo.sv_friction->GetFloat( ) * g_cl.m_local->m_surfaceFriction( );

	for( ; ticks_to_stop < g_cl.m_max_lag; ++ticks_to_stop ) {
		// calculate speed.
		float speed = velocity.length( );

		// if too slow return.
		if( speed <= 0.1f )
			break;

		// bleed off some speed, but if we have less than the bleed, threshold, bleed the threshold amount.
		float control = std::max( speed, g_csgo.sv_stopspeed->GetFloat( ) );

		// calculate the drop amount.
		float drop = control * friction * g_csgo.m_globals->m_interval;

		// scale the velocity.
		float newspeed = std::max( 0.f, speed - drop );

		if( newspeed != speed ) {
			// determine proportion of old speed we are using.
			newspeed /= speed;

			// adjust velocity according to proportion.
			velocity *= newspeed;
		}
	}

	const float local_vel = g_cl.m_local->m_vecVelocity().length_2d();
	float max_speed = std::floor(0.2f * (g_cl.m_local->m_bIsScoped() ? wpn_data->m_max_player_speed_alt : wpn_data->m_max_player_speed));


	// zero forwardmove and sidemove.
	if( ticks_to_stop > ( ( max - 1 ) - g_csgo.m_cl->m_choked_commands ) 
		|| !g_csgo.m_cl->m_choked_commands 
		|| local_vel >= max_speed 
		|| g_csgo.m_cl->m_choked_commands - 2 > max ) {

		if (local_vel <= 25.f)
			g_cl.m_cmd->m_forward_move = g_cl.m_cmd->m_side_move = 0.f;
		else
			NullVelocity();
	}
}

void Movement::FastStop() {


	if ( g_cl.m_pressing_move || !g_menu.main.misc.fast_stop.get() ) 
		return;
	
	if (!(g_cl.m_flags & FL_ONGROUND) || (g_cl.m_cmd->m_buttons & IN_JUMP))
		return;


	const float local_vel = g_cl.m_local->m_vecVelocity().length_2d();


	if (local_vel > 25.f)
		NullVelocity();
	else
		g_cl.m_cmd->m_side_move = g_cl.m_cmd->m_forward_move = 0.f;

}
