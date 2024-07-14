#include "includes.h"

Aimbot g_aimbot{ };;

vec3_t Aimbot::UpdateShootPosition(float pitch)
{

	if (!g_cl.m_local || !g_cl.m_processing)
		return vec3_t(0, 0, 0);

	if (g_csgo.m_cl->m_delta_tick <= 0)
		return g_cl.m_shoot_pos; // force latest shootpos to avoid crashes ig

	if (!g_cl.m_setupped || !g_cl.m_updated_values)
		return g_cl.m_local->m_vecOrigin() + g_cl.m_local->m_vecViewOffset();


	auto anim_state = g_cl.m_local->m_PlayerAnimState();

	g_cl.backup_count = g_cl.m_local->m_BoneCache().m_CachedBoneCount;

	// backup data
	vec3_t backup_abs_origin = g_cl.m_local->GetAbsOrigin();
	ang_t backup_abs_ang = g_cl.m_local->GetAbsAngles();

	const float backup_pitch = g_cl.m_poses[12];
	g_cl.m_poses[12] = (pitch + 90.f) / 180.f;

	g_cl.m_local->GetPoseParameters(g_cl.backup_poses);
	g_cl.m_local->GetAnimLayers(g_cl.backup_layers);

	// apply data to latest netwoked
	g_cl.m_local->SetPoseParameters(g_cl.m_poses);
	g_cl.m_local->SetAnimLayers(g_cl.m_layers);
	g_cl.m_local->SetAbsOrigin(g_cl.m_local->m_vecOrigin());
	g_cl.m_local->SetAbsAngles(ang_t(0, g_cl.m_abs_yaw, 0));

	// s/o unknowncheats $_$
	vec3_t result = g_cl.m_local->m_vecOrigin() + g_cl.m_local->m_vecViewOffset();

	// disable eye pos interpolation
	g_cl.m_local->m_fFlags() |= 0xF0;

	// null out incorrect data
	g_cl.m_local->some_ptr() = nullptr;

	// get eye pos
	g_cl.m_local->GetEyePos(&result);

	// fix compression
	g_cl.m_shoot_pos *= 0.974f;

	// normalize like on server
	g_cl.m_shoot_pos.normalize();

	// setupbones
	if (g_bone_handler.SetupBonesOnetap(g_cl.m_local, g_cl.m_matrix, false))
		// modify eye pos
		g_cl.m_local->ModifyEyePosition(anim_state, result, g_cl.m_local_bones);

	// restore data
	g_cl.m_poses[12] = backup_pitch;
	g_cl.m_local->SetAbsOrigin(backup_abs_origin);
	g_cl.m_local->SetAbsAngles(backup_abs_ang);
	g_cl.m_local->SetPoseParameters(g_cl.backup_poses);
	g_cl.m_local->SetAnimLayers(g_cl.backup_layers);

	// exit
	return result;
}

void AimPlayer::resetup_velocity(LagRecord* record, LagRecord* previous) {

	// reset all our velocity values
	record->m_velocity = record->m_pred_velocity = m_player->m_vecVelocity();

	// do not correct animations if not needed / unable to
	if (record->m_lag > 19)
		return;

	// if in air, do not correct velocity
	if (!(record->m_flags & FL_ONGROUND))
		return;

	// if previous was in air, he most likely landed on this tick, do not correct
	if (previous && !(previous->m_pred_flags & FL_ONGROUND))
		return;

	// if previous weapon pointer and current differs,
	// do not correct as this could impact alive loop rate & cycle
	if (previous && previous->m_weapon != record->m_weapon)
		return;

	// if no valid move rate, reset animation and do not correct
	if (record->m_layers[6].m_playback_rate == 0.f) {

		// this will be handled by fakewalk func
		// record->m_velocity = record->m_pred_velocity = vec3_t( 0, 0, 0 );
		return;
	}

	// if we have no previous record to correct with, exit correction
	if (!previous)
		return;

	// get animation lag in time
	const float anim_lag_time = game::TICKS_TO_TIME(record->m_anim_lag);

	// if in bounds, correct our velocity
	if (record->m_lag > 1 && anim_lag_time > 0.f && anim_lag_time < 1.f)
		record->m_velocity = (record->m_origin - previous->m_origin) / anim_lag_time;

	// store our average vel
	const float average_vel = record->m_velocity.length_2d();

	// if alive loop rate is the same, start correcting
	if (record->m_layers[11].m_playback_rate == previous->m_layers[11].m_playback_rate && average_vel > 1.f) {

		// get animation vel 
		const float animation_vel = (0.55f - ((record->m_layers[11].m_weight - 1.f) * 0.35f)) * record->m_max_speed;
		const float animation_weight = record->m_layers[11].m_weight;

		// make sure we do not break anything by checking if values go out of bounds or not
		if ((animation_weight >= 1.f && average_vel > animation_vel)
			|| (animation_weight < 1.f && (animation_vel >= average_vel || animation_weight > 0.f))) {

			// unscale our average vel and scale back to our weight velocity
			record->m_velocity.x = (record->m_velocity.x / average_vel) * animation_vel;
			record->m_velocity.y = (record->m_velocity.y / average_vel) * animation_vel;
		}
	}
}

void AimPlayer::correct_landing(LagRecord* record, LagRecord* previous) {


	// reset ground state
	record->m_ground_state = 0;

	if (record->m_lag <= 1 || !previous)
		return;

	// get animation lag in time
	const float anim_time = record->m_sim_time - game::TICKS_TO_TIME(record->m_anim_lag);

	// if current on ground
	if (record->m_flags & FL_ONGROUND) {

		// and previous was in air
		if (!(previous->m_flags & FL_ONGROUND)) {

			// get land cycle
			const float land_cycle = record->m_layers[5].m_cycle;

			// was land cycle triggered ?
			if (land_cycle < 0.999f) {

				// get land rate
				const float land_rate = record->m_layers[5].m_playback_rate;

				// if land rate is valid
				if (land_rate > 0.f) {

					// calc land time
					const float land_time = record->m_sim_time - (land_cycle / land_rate);

					// if land time hasnt expired yet it means he just landed
					if (land_time > anim_time && record->m_sim_time <= land_time) {
						// record->m_on_ground_time = land_time;
						record->m_ground_state = 1;
					}
				}
			}
		}
	}
	// if current is in air
	else {

		// and previous is also in air
		if (!(previous->m_flags & FL_ONGROUND)) {

			// get air cycle
			const float air_cycle = record->m_layers[4].m_cycle;

			// check if air cycle is valid
			if (air_cycle < 0.999f) {

				// get air rate
				const float air_rate = record->m_layers[4].m_playback_rate;

				// if air rate is valid
				if (air_rate > 0.f) {

					// calc air time
					const float air_time = record->m_sim_time - (air_cycle / air_rate);

					// if air time hasnt expired that means hes in air
					if (air_time > anim_time && record->m_sim_time <= air_time) {
						// record->m_on_ground_time = air_time;
						record->m_ground_state = 2;
					}
				}
			}
		}
	}


	if (record->m_ground_state > 0) {

		m_player->m_fFlags() &= ~FL_ONGROUND;

		if (record->m_ground_state == 2)
			m_player->m_fFlags() |= FL_ONGROUND;
	}
}

// NOTE: this could be slightly inaccurate
void AimPlayer::pre_anim_update(LagRecord* record, LagRecord* previous) {

	CCSGOPlayerAnimState* anim_state = m_player->m_PlayerAnimState();
	if (!anim_state)
		return;

	// apply uninterpolated origin
	m_player->SetAbsOrigin(m_player->m_vecOrigin());

	// do we have a previous record to work with?
	if (previous) {

		// credits: onetap v2 / llama
		if ((record->m_flags & FL_ONGROUND) && (previous->m_flags & FL_ONGROUND)) {
			anim_state->m_on_ground = true;
			anim_state->m_landing = false;
			anim_state->m_time_since_in_air = 0.f;
			m_player->m_flPoseParameter()[6] = 0.f;
		}

		// NOTE: im not sure of this so I will let it commented out
		// NOTE2: uncomment it out if you wanna test the difference
		m_player->SetAnimLayers(previous->m_layers);

		// make animstate values sync with server values
		// NOTE: skeet might be using previous->m_server_layers here instead of current
		// NOTE2: replace 'record->m_server_layers' by 'previous->m_server_layers' if you feel like its missing due to anim on walking people
		anim_state->m_move_weight = record->m_layers[6].m_weight;
		anim_state->m_primary_cycle = record->m_layers[6].m_cycle;
		anim_state->m_strafe_weight = record->m_layers[7].m_weight;
		anim_state->m_strafe_sequence = record->m_layers[7].m_sequence;
		anim_state->m_strafe_cycle = record->m_layers[7].m_cycle;
		anim_state->m_acceleration_weight = record->m_layers[12].m_weight;

		// set animstate element to previously networked values
		anim_state->m_foot_yaw = previous->m_foot_yaw;
		anim_state->m_move_yaw = previous->m_move_yaw;
		anim_state->m_move_yaw_cur_to_ideal = previous->m_move_yaw_cur_to_ideal;
		anim_state->m_move_yaw_ideal = previous->m_move_yaw_ideal;
		anim_state->m_move_weight_smoothed = previous->m_move_weight_smoothed;
	}
	else {

		// credits: onetap v2 / llama
		if (record->m_flags & FL_ONGROUND) {
			anim_state->m_on_ground = true;
			anim_state->m_landing = false;
			anim_state->m_time_since_in_air = 0.f;
			m_player->m_flPoseParameter()[6] = 0.f;
		}

		// apply layers to networked ones
		// NOTE: not sure if its any useless but oh well
		m_player->SetAnimLayers(record->m_layers);

		anim_state->m_primary_cycle = record->m_layers[6].m_cycle;
		anim_state->m_move_weight = record->m_layers[6].m_weight;
		anim_state->m_strafe_weight = record->m_layers[7].m_weight;
		anim_state->m_strafe_sequence = record->m_layers[7].m_sequence;
		anim_state->m_strafe_cycle = record->m_layers[7].m_cycle;
		anim_state->m_acceleration_weight = record->m_layers[12].m_weight;
		anim_state->m_last_update_time = record->m_sim_time - g_csgo.m_globals->m_interval;
	}


}

void AimPlayer::post_anim_update(LagRecord* record) {

	CCSGOPlayerAnimState* anim_state = m_player->m_PlayerAnimState();
	if (!anim_state)
		return;

	// store new values for next tick
	record->m_foot_yaw = anim_state->m_foot_yaw;
	record->m_move_yaw = anim_state->m_move_yaw;
	record->m_move_yaw_cur_to_ideal = anim_state->m_move_yaw_cur_to_ideal;
	record->m_move_yaw_ideal = anim_state->m_move_yaw_ideal;
	record->m_move_weight_smoothed = anim_state->m_move_weight_smoothed;

	// restore layers back to networked values
	m_player->SetAnimLayers(record->m_layers);

	// store updated/animated poses and rotation in lagrecord.
	this->m_player->GetPoseParameters(record->m_poses);
	record->m_abs_ang = ang_t(0.f, anim_state->m_foot_yaw, 0.f);
}

void AimPlayer::handle_fakewalk(LagRecord* record) {

	// reset fakewalk state
	record->m_fake_walk = false;

	// safety check
	if (!(record->m_flags & FL_ONGROUND)  // cant fakewalk in air
		|| record->m_anim_velocity.length_2d() <= 0.1f // cant fakewalk on stand
		|| record->m_anim_velocity.length_2d() > record->m_max_speed * 0.34f // cant fakewalk if above leg slide velocity
		|| (record->m_lag <= 3 && record->m_anim_lag <= 3)) // too low fake lag cant trigger fakewalk
		return;

	// this should be correct
	if (record->m_velocity.length() > 0.1f
		&& record->m_layers[6].m_weight == 0.0f
		&& record->m_layers[12].m_weight == 0.0f
		&& record->m_layers[6].m_playback_rate < 0.0001f)
		record->m_fake_walk = true;



	if (record->m_fake_walk)
		record->m_anim_velocity = record->m_velocity = record->m_pred_velocity = vec3_t(0, 0, 0);
}

void AimPlayer::handle_animations(LagRecord* record) {

	CCSGOPlayerAnimState* anim_state = this->m_player->m_PlayerAnimState();
	if (!anim_state)
		return;

	// player respawned.
	if (this->m_player->m_flSpawnTime() != this->m_spawn) {

		// reset animation state.
		game::ResetAnimationState(anim_state);

		// note new spawn time.
		this->m_spawn = m_player->m_flSpawnTime();
	}

	// backup global vars.
	const float backup_curtime = g_csgo.m_globals->m_curtime;
	const float backup_realtime = g_csgo.m_globals->m_realtime;
	const float backup_frametime = g_csgo.m_globals->m_frametime;
	const float backup_abs_frametime = g_csgo.m_globals->m_abs_frametime;
	const float backup_interp = g_csgo.m_globals->m_interp_amt;
	const int backup_framecount = g_csgo.m_globals->m_frame;
	const int backup_tickcount = g_csgo.m_globals->m_tick_count;

	// get simulation time and ticks.
	// NOTE: 2018 - 2021 skeet used m_player->m_flOldSimulationTim( ) + g_csgo.m_globals->m_interval;
	// const float time{ m_player->m_flSimulationTime( ) };
	const float time{ m_player->m_flOldSimulationTime() + g_csgo.m_globals->m_interval };
	const int ticks = game::TIME_TO_TICKS(time);

	// correct time and frametime to match server simulation.
	g_csgo.m_globals->m_curtime = time;
	g_csgo.m_globals->m_realtime = time;
	g_csgo.m_globals->m_frametime = g_csgo.m_globals->m_interval;
	g_csgo.m_globals->m_abs_frametime = g_csgo.m_globals->m_interval;
	g_csgo.m_globals->m_frame = ticks;
	g_csgo.m_globals->m_tick_count = ticks;
	g_csgo.m_globals->m_interp_amt = 0.f;

	// backup players netvars
	const vec3_t backup_abs_origin = m_player->GetAbsOrigin();
	const vec3_t backup_velocity = m_player->m_vecVelocity();
	const vec3_t backup_abs_velocity = m_player->m_vecAbsVelocity();
	const int backup_flags = m_player->m_fFlags();
	const int backup_eflags = m_player->m_iEFlags();
	const float backup_duck = m_player->m_flDuckAmount();

	LagRecord* previous = nullptr;

	// get previous record.
	if (this->m_records.size() >= 2)
		previous = this->m_records[1].get()->dormant() ? nullptr : this->m_records[1].get();

	// is player a bot?
	//bool bot = game::IsFakePlayer(this->m_player->index());

	// reset fakewalk state.
	record->m_fake_flick = false;
	record->m_mode = Resolver::Modes::RESOLVE_NONE;
	record->m_resolver_mode = XOR("NONE");
	record->m_extrapolated = record->m_broke_lc = false;
	record->m_ground_for_two_ticks = record->m_flags & FL_ONGROUND;

	// do we have a valid weapon?
	if (record->m_weapon) {

		// get data of that weapon
		WeaponInfo* wpn_data = record->m_weapon->GetWpnData();

		// get weapon max speed
		if (wpn_data)
			record->m_max_speed = this->m_player->m_bIsScoped() ? wpn_data->m_max_player_speed_alt : wpn_data->m_max_player_speed;
	}

	// get lag using networked simulation times
	record->m_anim_lag = record->m_lag = game::TIME_TO_TICKS(m_player->m_flSimulationTime() - m_player->m_flOldSimulationTime());

	// NOTE: this could have some inaccuracy
	// do we have previous data to work with?
	if (previous) {

		// set this
		record->m_ground_for_two_ticks = (record->m_flags & FL_ONGROUND) && (previous->m_flags & FL_ONGROUND);

		// apply anim lag using our 2 animation records
		record->m_anim_lag = game::TIME_TO_TICKS(record->m_sim_time - previous->m_sim_time);

		// make sure animticks doesnt go out of bounds
		if (record->m_anim_lag > 32 || previous->m_sim_time == 0.f)
			record->m_anim_lag = 1;

		// get current and previous alive loop rate
		float current_rate = record->m_layers[11].m_playback_rate;
		float previous_rate = previous->m_layers[11].m_playback_rate;

		// check if the rates are available to correct with
		bool should_correct = current_rate > 0.f && previous_rate > 0.f;

		// if weapon has the same pointer and alive loop rate is valid
		// start correcting fakelag
		if (should_correct && record->m_weapon == previous->m_weapon) {

			// get current and previous alive loop cycle
			float current_cycle = record->m_layers[11].m_cycle;
			float previous_cycle = previous->m_layers[11].m_cycle;

			// reset animation lag
			record->m_anim_lag = 0;

			// if previous cycle is bigger than current
			// give our current cycle some value to correct with
			if (previous_cycle > current_cycle)
				current_cycle += 1.f;

			// start our correction loop
			while (current_cycle > previous_cycle) {

				// store a backup for our anim ticks
				const int backup_anim_ticks = record->m_anim_lag;

				// calculate next rate in time
				const float future_rate = g_csgo.m_globals->m_interval * previous_rate;

				// apply it to our previous cycle
				previous_cycle += future_rate;

				// if cycle goes out of bounds
				if (previous_cycle >= 1.f)
					previous_rate = record->m_layers[11].m_playback_rate; // force our previous rate to be current rate

				// add up a lag tick
				++record->m_anim_lag;

				// check if we should restore our animticks to our backup ticks or not
				if (previous_cycle > current_cycle && (previous_cycle - current_cycle) > (future_rate * 0.5f))
					record->m_anim_lag = backup_anim_ticks;
			}
		}
	}

	// clamp our anim ticks to make sure we do not go out of bounds
	if (record->m_anim_lag < record->m_lag || record->m_anim_lag > 19)
		record->m_anim_lag = record->m_lag;

	// run our velocity correction
	// NOTE: pasted from something else than skeet for now
	resetup_velocity(record, previous);

	// correct landing animation
	correct_landing(record, previous);

	// fix gravity
	if (m_player->m_fFlags() & FL_ONGROUND)
		record->m_anim_velocity.z = record->m_velocity.z = record->m_pred_velocity.z = 0.f;

	// handle fakewalk
	handle_fakewalk(record);

	// set anim vel to corrected velocity
	record->m_anim_velocity = record->m_velocity;

	// ghetto check to see if thye're fakelagging or not
	if (g_menu.main.aimbot.correct.get()) // !bot && 
		g_resolver.ResolveAngles(m_player, record);

	this->m_player->m_vecOrigin() = record->m_origin;
	this->m_player->m_vecVelocity() = this->m_player->m_vecAbsVelocity() = record->m_velocity;
	this->m_player->m_iEFlags() &= ~(EFL_DIRTY_ABSTRANSFORM | EFL_DIRTY_ABSVELOCITY);
	this->m_player->m_eye_angles() = record->m_eye_angles;

	// fix animating in same frame
	anim_state->m_last_update_frame = g_csgo.m_globals->m_frame - 1;

	// run our pre anim update
	pre_anim_update(record, previous);

	g_hooks.m_bUpdatingCSA[m_player->index()] = this->m_player->m_bClientSideAnimation() = true;
	m_player->UpdateClientSideAnimation();
	g_hooks.m_bUpdatingCSA[m_player->index()] = this->m_player->m_bClientSideAnimation() = false;

	// run our post anim update
	post_anim_update(record);

	// tell the game animations have changed
	this->m_player->InvalidatePhysicsRecursive(InvalidatePhysicsBits_t::ANIMATION_CHANGED);

	// restore globals.
	g_csgo.m_globals->m_realtime = backup_realtime;
	g_csgo.m_globals->m_curtime = backup_curtime;
	g_csgo.m_globals->m_frametime = backup_frametime;
	g_csgo.m_globals->m_abs_frametime = backup_abs_frametime;
	g_csgo.m_globals->m_frame = backup_framecount;
	g_csgo.m_globals->m_tick_count = backup_tickcount;
	g_csgo.m_globals->m_interp_amt = backup_interp;

	// setup bones for this record
	// record->m_setup = g_bone_handler.SetupBones( m_player, record->m_bones, record->m_sim_time );
	record->m_setup = g_bone_handler.SetupBonesOnetap(m_player, record->m_bones, false);

	// restore backup data.
	m_player->m_vecVelocity() = backup_velocity;
	m_player->m_vecAbsVelocity() = backup_abs_velocity;
	m_player->m_fFlags() = backup_flags;
	m_player->m_iEFlags() = backup_eflags;
	m_player->m_flDuckAmount() = backup_duck;
}

bool AimPlayer::handle_simulation(Player* player) {

	// store networked simtimes
	const float current_sim_time = player->m_flSimulationTime();
	const float previous_sim_time = player->m_flOldSimulationTime();

	// no simulation data update from server -> dont update
	if (current_sim_time == previous_sim_time)
		return false;

	// detect if they're shifting (for visuals)
	// note maybe replace -2 by -game::TIME_TO_TICKS( current_sim_time - previous_sim_time )
	m_shift = (game::TIME_TO_TICKS(current_sim_time) - g_csgo.m_globals->m_tick_count) < -2;

	// get current alive loop cycle & rate
	const float current_alive_loop_cycle = player->m_AnimOverlay()[11].m_cycle;
	const float current_alive_loop_rate = player->m_AnimOverlay()[11].m_playback_rate;

	// if server layers havent updated, fix simulation data by setting them to last networked
	// note: onetap does not check for playbackrate, only cycle in v3 & v4
	// note2: they also use previous record layer values instead of storing them manually
	if (current_alive_loop_cycle == this->m_alive_loop_cycle
		&& current_alive_loop_rate == this->m_alive_loop_rate) {


		// fix simulation time by forcing it to not update
		player->m_flSimulationTime() = previous_sim_time;
		return false;
	}

	// store last valid simulation data
	m_cur_sim = current_sim_time;
	m_old_sim = previous_sim_time;
	m_alive_loop_cycle = current_alive_loop_cycle;
	m_alive_loop_rate = current_alive_loop_rate;

	// tell the cheat we should update his record
	return true;
}

void AimPlayer::on_data_update(Player* player) {
	bool reset = (!g_menu.main.aimbot.enable.get() || player->m_lifeState() == LIFE_DEAD || !player->enemy(g_cl.m_local));
	bool disable = (!reset && !g_cl.m_processing);

	// if this happens, delete all the lagrecords.
	if (reset) {
		player->m_bClientSideAnimation() = true;
		m_records.clear();

		if (player)
			g_hooks.m_bUpdatingCSA[player->index()] = true;
		return;
	}

	// just disable anim if this is the case.
	if (disable) {
		player->m_bClientSideAnimation() = true;
		return;
	}

	// update player ptr if required.
	// reset player if changed.
	if (m_player != player) {
		m_uses_secreto = false;
		m_has_whitelist_on = false;
		m_last_rate = m_last_cycle = -1.f;
		m_records.clear();
	}

	// update player ptr.
	m_player = player;

	// indicate that this player has been out of pvs.
	// insert dummy record to separate records
	// to fix stuff like animation and prediction.
	if (player->dormant()) {
		bool insert = true;

		if (m_records.empty() || !m_records.front()->dormant())
			add_record();

		while (m_records.size() > 2)
			m_records.pop_back();

		return;
	}

	if (!handle_simulation(player))
		return;

	// get reference to newly added record.
	LagRecord* current = add_record();

	// update animations on current record.
	// call resolver.
	handle_animations(current);

	// set shifting tickbase record.
	current->m_shift = game::TIME_TO_TICKS(current->m_sim_time) - g_csgo.m_globals->m_tick_count;

	while (this->m_records.size() > 64)
		this->m_records.pop_back();
}

void AimPlayer::OnRoundStart(Player* player) {
	m_player = player;
	m_shots = 0;
	m_missed_shots = 0;

	// reset stand and body index.
	m_old_stand_move_idx = 0;
	m_old_stand_no_move_idx = 0;
	m_body_idx = 0;
	m_body_pred_idx = 0;
	m_air_idx = 0;
	m_stand_move_idx = m_stand_no_move_idx = 0;
	m_walk_record.m_sim_time = -1.f;
	m_update_captured = 0;
	m_first_delta = 90.f;
	m_overlap_offset = 0.f;
	m_body_timer = FLT_MAX;
	m_has_updated = 0;
	m_last_body = FLT_MIN;
	m_update_count = 0;
	m_upd_time = FLT_MIN;
	m_moved = false;
	m_last_time = m_last_rate = m_last_cycle = -1.f;
	m_change_stored = 0;
	m_uses_secreto = false;
	m_has_whitelist_on = false;
	m_last_prev_ground = true;
	m_last_prev_ladder = false;
	m_missed_invertfs = false;
	m_missed_back = false;
	m_last_duration_in_air = 0.f;
	m_ticks_since_dormant = INT_MAX;


	m_records.clear();
	m_hitboxes.clear();

	// IMPORTANT: DO NOT CLEAR LAST HIT SHIT.
}

void AimPlayer::SetupHitboxes(LagRecord* record, bool history) {


	// reset hitboxes.
	m_hitboxes.clear();
	m_prefer_body = false;

	// attempt of fixing lc bullshit
	if (!record->valid())
		return;


	if (g_cl.m_weapon_id == ZEUS) {
		// hitboxes for the zeus.
		m_hitboxes.push_back({ HITBOX_BODY });
		m_hitboxes.push_back({ HITBOX_CHEST });
		return;
	}

	m_prefer_body = g_menu.main.aimbot.prefer_baim.get();

	if (m_prefer_body) {

		if (g_menu.main.aimbot.prefer_baim_disablers.get(0) && record->m_mode == Resolver::Modes::RESOLVE_LBY_PRED)
			m_prefer_body = false;

		if (g_menu.main.aimbot.prefer_baim_disablers.get(1) && record->m_mode == Resolver::Modes::RESOLVE_WALK)
			m_prefer_body = false;

		if (g_menu.main.aimbot.prefer_baim_disablers.get(2) && (m_is_kaaba || m_is_cheese_crack))
			m_prefer_body = false;

		if (g_menu.main.aimbot.prefer_baim_disablers.get(3) && std::abs(math::AngleDiff(record->m_eye_angles.y, record->m_body)) <= 10.f)
			m_prefer_body = false;
	}

	// only, on key.
	MultiDropdown hitbox{ g_menu.main.aimbot.hitbox };


	// head
	if (hitbox.get(0) && !g_aimbot.m_force_body) {
		m_hitboxes.push_back({ HITBOX_HEAD });
	}

	// stomach
	if (hitbox.get(2)) {
		m_hitboxes.push_back({ HITBOX_BODY, NORMAL });
	}

	// chest
	if (hitbox.get(1)) {
		m_hitboxes.push_back({ HITBOX_CHEST, NORMAL });


		if (!g_aimbot.m_force_body)
			m_hitboxes.push_back({ HITBOX_UPPER_CHEST });
	}


	if (hitbox.get(2)) {
		m_hitboxes.push_back({ HITBOX_PELVIS, NORMAL });
	}

	if (hitbox.get(1)) {
		if (!g_aimbot.m_force_body)
			m_hitboxes.push_back({ HITBOX_UPPER_CHEST });
	}

	// legs.
	if (hitbox.get(3)) {
		m_hitboxes.push_back({ HITBOX_L_CALF });
		m_hitboxes.push_back({ HITBOX_R_CALF });

		m_hitboxes.push_back({ HITBOX_L_THIGH });
		m_hitboxes.push_back({ HITBOX_R_THIGH });
	}

	// feet.
	if (hitbox.get(4)) {
		m_hitboxes.push_back({ HITBOX_L_FOOT });
		m_hitboxes.push_back({ HITBOX_R_FOOT });
	}

}

void Aimbot::init() {
	// clear old targets.
	m_targets.clear();

	m_total_scanned = 0;
	m_target = nullptr;
	m_aim = vec3_t{ };
	m_angle = ang_t{ };
	m_damage = 0.f;
	m_record = nullptr;
	m_stop = false;
}

void Aimbot::StripAttack() {
	if (g_cl.m_weapon_id == REVOLVER)
		g_cl.m_cmd->m_buttons &= ~IN_ATTACK2;

	else
		g_cl.m_cmd->m_buttons &= ~IN_ATTACK;
}

void Aimbot::StartTargetSelection() {
	if (!g_cl.m_processing)
		return;

	LagRecord* front{ };
	AimPlayer* data{ };

	// setup bones for all valid targets.
	for (int i{ 1 }; i <= g_csgo.m_globals->m_max_clients; ++i) {
		Player* player = g_csgo.m_entlist->GetClientEntity< Player* >(i);

		if (!IsValidTarget(player))
			continue;

		data = &m_players[i - 1];

		// we have no data, or the player ptr in data is invalid
		if (!data || !data->m_player || data->m_player->index() != player->index())
			continue;

		data->m_hit = false;

		// mostly means he just went out of dormancy
		if (data->m_records.size() <= 1)
			continue;

		// get our front record
		front = data->m_records.front().get();

		// front record is invalid, skip this player
		if (!front || front->dormant() || front->immune() || !front->m_setup)
			continue;

		if (g_menu.main.misc.whitelist.get()) {

			if (data->m_is_secreto
				&& !data->m_is_godhook
				&& !data->m_is_robertpaste
				&& !data->m_is_pandora
				&& !data->m_is_fade
				&& !data->m_is_dopium
				&& !data->m_is_cheese_crack
				&& !data->m_is_kaaba)
				continue;
		}

		// store player as potential target this tick.
		m_targets.emplace_back(data);
	}
}

void Aimbot::FinishTargetSelection() {
	if (!g_cl.m_processing)
		return;

	static auto sort_targets = [&](const AimPlayer* a, const AimPlayer* b) {
		// player b and player a are the same
		// do nothing
		if (a == b)
			return false;

		// player a is invalid, if player b is valid, prioritize him
		// else do nothing
		if (!a)
			return b ? true : false;

		// player b is invalid, if player a is valid, prioritize him
		// else do nothing
		if (!b)
			return a ? true : false;

		// this is the same player
		// in that case, do nothing
		if (a->m_player == b->m_player || a->m_player->index() == b->m_player->index())
			return false;

		// get fov of player a
		float fov_a = math::GetFOV(g_cl.m_view_angles, g_cl.m_shoot_pos, a->m_player->WorldSpaceCenter());

		// get fov of player b
		float fov_b = math::GetFOV(g_cl.m_view_angles, g_cl.m_shoot_pos, b->m_player->WorldSpaceCenter());

		// if player a fov lower than player b fov prioritize him
		return fov_a < fov_b;
	};

	// if we have only 1 targets or less, no need to sort
	if (m_targets.size() <= 1)
		return;

	// std::execution::par -> parallel sorting (multithreaded)
	// NOTE: not obligated, std::sort doesnt take alot of cpu power but its still better
	std::sort(std::execution::par, m_targets.begin(), m_targets.end(), sort_targets);

	// target limit based on our prioritized targets
	while (this->m_targets.size() > g_menu.main.aimbot.target_limit.get())
		this->m_targets.pop_back();
}

void Aimbot::think() {
	// do all startup routines.
	init();

	// sanity.
	if (!g_cl.m_weapon || !g_cl.m_weapon_info)
		return;

	// no grenades or bomb.
	if (g_cl.m_weapon_type == WEAPONTYPE_GRENADE || g_cl.m_weapon_type == WEAPONTYPE_C4)
		return;

	if (!g_cl.m_weapon_fire)
		StripAttack();

	// we have no aimbot enabled.
	if (!g_menu.main.aimbot.enable.get())
		return;

	// animation silent aim, prevent the ticks with the shot in it to become the tick that gets processed.
	// we can do this by always choking the tick before we are able to shoot.
	bool revolver = g_cl.m_weapon_id == REVOLVER && g_cl.m_revolver_cock != 0;

	// one tick before being able to shoot.
	if (revolver && g_cl.m_revolver_cock > 0 && g_cl.m_revolver_cock == g_cl.m_revolver_query) {
		*g_cl.m_packet = false;
		return;
	}



	// only do that on semi auto  snipers
	if (!g_cl.m_weapon_fire && (g_cl.m_weapon_id == SSG08 || g_cl.m_weapon_id == AWP))
		return StripAttack();



	StartTargetSelection();

	// run knifebot.
	if (g_cl.m_weapon_type == WEAPONTYPE_KNIFE && g_cl.m_weapon_id != ZEUS) {

		// no point in aimbotting if we cannot fire this tick.
		if (g_cl.m_weapon_fire)
			knife();

		return;
	}

	FinishTargetSelection();


	// scan available targets... if we even have any.
	find();


	// we have a normal weapon or a non cocking revolver
	// choke if its the processing tick.
	if (g_cl.m_weapon_fire && !g_cl.m_lag && !revolver && g_menu.main.antiaim.fakelag_silent_aim.get()) {
		*g_cl.m_packet = false;
		StripAttack();
		return;
	}

	if (!g_cl.m_weapon_fire)
		return StripAttack();

	// finally set data when shooting.
	apply();
}

void Aimbot::find() {
	struct BestTarget_t { Player* player{}; vec3_t pos{}; float damage{}; int hitbox{}; int hitgroup{}; LagRecord* record{}; };

	vec3_t       tmp_pos;
	float        tmp_damage;
	int			 tmp_hitbox{}, tmp_hitgroup{};
	BestTarget_t best;
	float best_health = 999.f;
	best.player = nullptr;
	best.damage = -1.f;
	best.pos = vec3_t{ };
	best.record = nullptr;
	best.hitbox = -1;
	best.hitgroup = -1;

	m_found_hit = false;

	if (m_targets.empty())
		return;

	// iterate all targets.
	for (const auto& t : m_targets) {
		if (t->m_records.empty())
			continue;

		const int hp = t->m_player->m_iHealth();

		// this player broke lagcomp.
		// his bones have been resetup by our lagcomp.
		// therfore now only the front record is valid.
		if (g_extrapolation.HandleLagCompensation(t)) {

			if (t->m_delayed)
				continue;

			LagRecord* front = t->m_records.front().get();

			t->SetupHitboxes(front, false);
			if (t->m_hitboxes.empty())
				continue;

			// rip something went wrong..
			if (t->GetBestAimPosition(tmp_pos, tmp_damage, tmp_hitbox, tmp_hitgroup, front)) {

				// if we made it so far, set shit.
				if (tmp_damage > best.damage || tmp_damage >= hp) {

					best.player = t->m_player;
					best.pos = tmp_pos;
					best.damage = tmp_damage;
					best.record = front;
					best.hitbox = tmp_hitbox;
					best.hitgroup = tmp_hitgroup;

					if (best.damage >= best.player->m_iHealth())
						break;
				}
			}
		}

		// player did not break lagcomp.
		// history aim is possible at this point.
		else {


			// ideal record
			LagRecord* ideal = g_resolver.FindIdealRecord(t);
			if (!ideal)
				continue;



			t->SetupHitboxes(ideal, false);
			if (t->m_hitboxes.empty())
				continue;

			bool hit_ideal = t->GetBestAimPosition(tmp_pos, tmp_damage, tmp_hitbox, tmp_hitgroup, ideal);

			// try to select best record as target.
			if (hit_ideal) {

				if (tmp_damage > best.damage || tmp_damage >= hp) {
					// if we made it so far, set shit.
					best.player = t->m_player;
					best.pos = tmp_pos;
					best.damage = tmp_damage;
					best.record = ideal;
					best.hitbox = tmp_hitbox;
					best.hitgroup = tmp_hitgroup;
					best_health = hp;

					if (best.damage >= best.player->m_iHealth())
						break;
				}
			}

			if (m_total_scanned >= m_targets.size() / 2)
				continue;

			// last record
			LagRecord* last = g_resolver.FindLastRecord(t);
			if (!last || last == ideal ||  // if last invalid or last is ideal
				last->m_origin.dist_to(ideal->m_origin) <= 0.1f  // if origin is very close
				&& std::abs(math::AngleDiff(last->m_eye_angles.y, ideal->m_eye_angles.y)) <= 5.f) // and angle isnt different
				continue; // skip backtrack

			t->SetupHitboxes(last, true);
			if (t->m_hitboxes.empty())
				continue;

			++m_total_scanned;

			// rip something went wrong..
			if (t->GetBestAimPosition(tmp_pos, tmp_damage, tmp_hitbox, tmp_hitgroup, last)) {

				if (tmp_damage > best.damage || tmp_damage >= hp) {
					best.player = t->m_player;
					best.pos = tmp_pos;
					best.damage = tmp_damage;
					best.record = last;
					best.hitbox = tmp_hitbox;
					best.hitgroup = tmp_hitgroup;
					best_health = hp;

					if (best.damage >= best.player->m_iHealth())
						break;
				}
			}
		}
	}



	const bool ground = (g_cl.m_flags & FL_ONGROUND) && g_cl.m_local->m_fFlags() & FL_ONGROUND;




	const bool found_target = best.player && best.record && best.damage > 0;

	// set autostop shit.
	// set autostop shit.
	if (found_target || m_found_hit && g_menu.main.aimbot.quick_stop_mode.get(4)) {
		if (ground && g_menu.main.aimbot.quick_stop.get()) {

			if (g_cl.m_weapon_fire || g_menu.main.aimbot.quick_stop_mode.get(1) && g_cl.m_player_fire)
				m_stop = true;

		}
	}

	// verify our target and set needed data->
	if (found_target) {


		//calculate aim angle.
		math::VectorAngles(best.pos - g_cl.m_shoot_pos, m_angle);

		// vec3_t new_shoot_pos{ UpdateShootPosition(m_angle.x) };

		// re-calculate aim angle.
		// math::VectorAngles(best.pos - new_shoot_pos, m_angle);


		// set member vars.
		m_target = best.player;
		m_aim = best.pos;
		m_damage = best.damage;
		m_record = best.record;
		m_hitbox = best.hitbox;
		m_hitgroup = best.hitgroup;


		// write data, needed for traces / etc.
		m_record->cache();


		bool on = g_menu.main.aimbot.hitchance.get() && !g_menu.main.aimbot.nospread.get();
		bool hit = on && CheckHitchance(m_target, m_hitbox, m_angle);


		// if we can scope.
		bool can_scope = !g_cl.m_local->m_bIsScoped() && (g_cl.m_weapon_id == AUG || g_cl.m_weapon_id == SG553 || g_cl.m_weapon_type == WEAPONTYPE_SNIPER_RIFLE);

		if (can_scope && ground) {
			// always.
			if (g_menu.main.aimbot.zoom.get() == 1) {
				g_cl.m_cmd->m_buttons |= IN_ATTACK2;
				g_cl.m_cmd->m_buttons &= ~IN_ATTACK;
				return;
			}
		}

		if (hit) {
			// right click attack.
			if (g_menu.main.aimbot.nospread.get() && g_cl.m_weapon_id == REVOLVER)
				g_cl.m_cmd->m_buttons |= IN_ATTACK2;

			// left click attack.
			else
				g_cl.m_cmd->m_buttons |= IN_ATTACK;
		}

	}
}

bool Aimbot::CheckHitchance(Player* player, int hitbox, const ang_t& angle) {


	if (!g_cl.m_weapon_fire || !g_cl.m_player_fire)
		return false;

	const model_t* model = m_record->m_player->GetModel();
	if (!model)
		return false;

	studiohdr_t* hdr = g_csgo.m_model_info->GetStudioModel(model);
	if (!hdr)
		return false;

	mstudiohitboxset_t* set = hdr->GetHitboxSet(m_record->m_player->m_nHitboxSet());
	if (!set)
		return false;

	mstudiobbox_t* bbox = set->GetHitbox(hitbox);
	if (!bbox)
		return false;

	constexpr float HITCHANCE_MAX = 100.f;
	constexpr int   SEED_MAX = 255;
	const float accurate_speed = std::floor((g_cl.m_local->m_bIsScoped() ? g_cl.m_weapon_info->m_max_player_speed_alt : g_cl.m_weapon_info->m_max_player_speed) * 0.33f);

	// force accuracy
	if (g_menu.main.aimbot.quick_stop_mode.get(2) && g_cl.m_local->m_vecVelocity().length_2d() >= accurate_speed)
		return false;

	vec3_t     start{ g_cl.m_shoot_pos }, end, fwd, right, up, dir, wep_spread;
	float      inaccuracy, spread;
	CGameTrace tr;
	float chance = g_menu.main.aimbot.hitchance_in_air.get() && !(g_cl.m_local->m_fFlags() & FL_ONGROUND) ? g_menu.main.aimbot.in_air_hitchance.get() : g_menu.main.aimbot.hitchance_amount.get();


	const int health = player->m_iHealth();
	float goal_damage = 1.f;

	if (g_menu.main.aimbot.accuracy_boost.get()) {

		// divide by 2 cus we use 255 seeds
		float lol_damage = std::clamp(m_damage, 1.f, (float)health);
		goal_damage = std::clamp(lol_damage * (g_menu.main.aimbot.accuracy_boost_amt.get() / 110.f), 1.f, m_damage);
	}


	if (g_cl.m_weapon_id == WEAPON_ZEUS) {
		chance = 70.f;
		goal_damage = health / 3.f;
	}


	// ghetto but idgaf
	float needed_hits = SEED_MAX * (chance / HITCHANCE_MAX);
	float current_hits = 0.f;

	// get needed directional vectors.
	math::AngleVectors(angle, &fwd, &right, &up);

	// store off inaccuracy / spread ( these functions are quite intensive and we only need them once ).
	inaccuracy = g_cl.m_weapon->GetInaccuracy();
	spread = g_cl.m_weapon->GetSpread();

	// iterate all possible seeds.
	for (int i{ 0 }; i <= SEED_MAX; i++) {
		// get spread.
		wep_spread = g_cl.m_weapon->CalculateSpread(i, inaccuracy, spread);

		// get spread direction.
		dir = (fwd + (right * wep_spread.x) + (up * wep_spread.y)).normalized();

		penetration::PenetrationInput_t in;

		in.m_damage = goal_damage;
		in.m_damage_pen = goal_damage;
		in.m_can_pen = true;
		in.m_target = player;
		in.m_from = g_cl.m_local;
		in.m_pos = start + (dir * ((m_aim - g_cl.m_shoot_pos).length_sqr() + 128.f));
		in.m_center = false;

		penetration::PenetrationOutput_t out;

		bool did_hit = penetration::run(&in, &out);

		if (did_hit && out.m_damage > 0.f)
			++current_hits;
	}

	if (!(g_cl.m_flags & FL_ONGROUND)
		&& g_menu.main.aimbot.quick_stop.get()
		&& g_menu.main.aimbot.quick_stop_mode.get(3)  // in air
		&& current_hits >= SEED_MAX * (g_menu.main.aimbot.quick_stop_air_sens.get() / 100.f)) { // min accuracy before autostopping
		m_stop_air = true;
	}

	m_hit_chance = (current_hits / SEED_MAX) * HITCHANCE_MAX;
	return current_hits >= needed_hits;
}

bool AimPlayer::SetupHitboxPoints(LagRecord* record, BoneArray* bones, int index, std::vector< vec3_t >& points) {
	// reset points.
	points.clear();

	const model_t* model = m_player->GetModel();
	if (!model)
		return false;

	studiohdr_t* hdr = g_csgo.m_model_info->GetStudioModel(model);
	if (!hdr)
		return false;

	mstudiohitboxset_t* set = hdr->GetHitboxSet(m_player->m_nHitboxSet());
	if (!set)
		return false;

	mstudiobbox_t* bbox = set->GetHitbox(index);
	if (!bbox)
		return false;

	// get hitbox scales.
	float head_scale_menu = g_menu.main.aimbot.head_scale.get() / 100.f;
	float body_scale_menu = g_menu.main.aimbot.body_scale.get() / 100.f;

	if (g_cl.m_weapon_id == ZEUS)
		body_scale_menu = head_scale_menu = 0.7f;

	// big inair fix.
	if (!(record->m_pred_flags & FL_ONGROUND))
		body_scale_menu = head_scale_menu = 0.5f;

	// these indexes represent boxes.
	if (bbox->m_radius <= 0.f) {
		// references: 
		//      https://developer.valvesoftware.com/wiki/Rotation_Tutorial
		//      CBaseAnimating::GetHitboxBonePosition
		//      CBaseAnimating::DrawServerHitboxes

		// convert rotation angle to a matrix.
		matrix3x4_t rot_matrix;
		g_csgo.AngleMatrix(bbox->m_angle, rot_matrix);

		// apply the rotation to the entity input space (local).
		matrix3x4_t matrix;
		math::ConcatTransforms(bones[bbox->m_bone], rot_matrix, matrix);

		// extract origin from matrix.
		vec3_t origin = matrix.GetOrigin();

		// compute raw center point.
		vec3_t center = (bbox->m_mins + bbox->m_maxs) / 2.f;

		// the feet hiboxes have a side, heel and the toe.
		if (index == HITBOX_R_FOOT || index == HITBOX_L_FOOT) {
			float d1 = (bbox->m_mins.z - center.z) * 0.875f;

			// invert.
			if (index == HITBOX_L_FOOT)
				d1 *= -1.f;

			// side is more optimal then center.
			points.push_back({ center.x, center.y, center.z + d1 });

			if (g_menu.main.aimbot.multipoint.get(4)) {
				// get point offset relative to center point
				// and factor in hitbox scale.
				float d2 = (bbox->m_mins.x - center.x) * body_scale_menu;
				float d3 = (bbox->m_maxs.x - center.x) * body_scale_menu;

				// heel.
				points.push_back({ center.x + d2, center.y, center.z });

				// toe.
				points.push_back({ center.x + d3, center.y, center.z });
			}
		}

		// nothing to do here we are done.
		if (points.empty())
			return false;

		// rotate our bbox points by their correct angle
		// and convert our points to world space.
		for (auto& p : points) {
			// VectorRotate.
			// rotate point by angle stored in matrix.
			p = { p.dot(matrix[0]), p.dot(matrix[1]), p.dot(matrix[2]) };

			// transform point to world space.
			p += origin;
		}
	}

	// these hitboxes are capsules.
	else {
		// factor in the pointscale.
		float head_scale = bbox->m_radius * head_scale_menu;
		float head_scale_backwards = bbox->m_radius * 0.65f;
		float body_scale = bbox->m_radius * body_scale_menu;

		// compute raw center point.
		vec3_t center = (bbox->m_mins + bbox->m_maxs) / 2.f;

		// head has 5 points.
		if (index == HITBOX_HEAD) {
			// add center.
			points.push_back(center);

			if (g_menu.main.aimbot.multipoint.get(0)) {
				// rotation matrix 45 degrees.
				// https://math.stackexchange.com/questions/383321/rotating-x-y-points-45-degrees
				// std::cos( deg_to_rad( 45.f ) )
				constexpr float rotation = 0.70710678f;

				// top/back 45 deg.
				// this is the best spot to shoot at.
				points.push_back({ bbox->m_maxs.x + (rotation * head_scale_backwards), bbox->m_maxs.y + (-rotation * head_scale_backwards), bbox->m_maxs.z });

				// right.
				points.push_back({ bbox->m_maxs.x, bbox->m_maxs.y, bbox->m_maxs.z + head_scale });

				// left.
				points.push_back({ bbox->m_maxs.x, bbox->m_maxs.y, bbox->m_maxs.z - head_scale });

				// back.
				points.push_back({ bbox->m_maxs.x, bbox->m_maxs.y - head_scale, bbox->m_maxs.z });

				// get animstate ptr.
				CCSGOPlayerAnimState* state = record->m_player->m_PlayerAnimState();

				// add this point only under really specific circumstances.
				// if we are standing still and have the lowest possible pitch pose.
				if (state && record->m_anim_velocity.length() <= 0.1f && record->m_eye_angles.x <= state->m_aim_pitch_min) {

					// bottom point.
					points.push_back({ bbox->m_maxs.x - head_scale, bbox->m_maxs.y, bbox->m_maxs.z });
				}
			}
		}

		// body has 5 points.
		else if (index == HITBOX_BODY) {
			// center.
			points.push_back(center);

			// back.
			if (g_menu.main.aimbot.multipoint.get(2))
				points.push_back({ center.x, bbox->m_maxs.y - body_scale, center.z });
		}

		else if (index == HITBOX_PELVIS || index == HITBOX_UPPER_CHEST) {
			// back.
			points.push_back({ center.x, bbox->m_maxs.y - body_scale, center.z });
		}

		// other stomach/chest hitboxes have 2 points.
		else if (index == HITBOX_THORAX || index == HITBOX_CHEST) {
			// add center.
			points.push_back(center);

			// add extra point on back.
			if (g_menu.main.aimbot.multipoint.get(1))
				points.push_back({ center.x, bbox->m_maxs.y - body_scale, center.z });
		}

		else if (index == HITBOX_R_CALF || index == HITBOX_L_CALF) {
			// add center.
			points.push_back(center);

			// half bottom.
			if (g_menu.main.aimbot.multipoint.get(3))
				points.push_back({ bbox->m_maxs.x - (bbox->m_radius / 2.f), bbox->m_maxs.y, bbox->m_maxs.z });
		}

		else if (index == HITBOX_R_THIGH || index == HITBOX_L_THIGH) {
			// add center.
			points.push_back(center);
		}

		// arms get only one point.
		else if (index == HITBOX_R_UPPER_ARM || index == HITBOX_L_UPPER_ARM) {
			// elbow.
			points.push_back({ bbox->m_maxs.x + bbox->m_radius, center.y, center.z });
		}

		// nothing left to do here.
		if (points.empty())
			return false;

		// transform capsule points.
		for (auto& p : points)
			math::VectorTransform(p, bones[bbox->m_bone], p);
	}

	return true;
}

bool AimPlayer::GetBestAimPosition(vec3_t& aim, float& damage, int& hitbox, int& hitgroup, LagRecord* record) {
	bool                  done, pen;
	float                 dmg, pendmg;
	HitscanData_t         scan;
	std::vector< vec3_t > points;


	if (g_menu.main.aimbot.prefer_accuracy.get(1) && record->m_broke_lc)
		return false;

	// get player hp.
	int hp = std::min(100, m_player->m_iHealth());

	if (g_cl.m_weapon_id == ZEUS) {
		dmg = pendmg = hp + 1;
		pen = false;
	}
	else {
		dmg = g_aimbot.m_damage_toggle ? g_menu.main.aimbot.override_dmg_value.get() : g_menu.main.aimbot.minimal_damage.get();

		if (dmg >= 100)
			dmg = pendmg = hp + (dmg - 100.f);

		pendmg = dmg;
		pen = true;
	}

	bool prefer_center = g_menu.main.aimbot.quick_stop.get() && g_menu.main.aimbot.quick_stop_mode.get(3) && !(g_cl.m_flags & FL_ONGROUND);

	// write all data of this record l0l.
	record->cache();

	const int delay_shot = g_menu.main.aimbot.delay_shot_center.get();

	// iterate hitboxes.
	for (const auto& it : m_hitboxes) {
		done = false;

		// setup points on hitbox.
		if (!SetupHitboxPoints(record, record->m_bones, it.m_index, points))
			continue;



		bool skip_this_hb = false;

		// iterate points on hitbox.
		for (const auto& point : points) {


			// g_csgo.m_debug_overlay->AddBoxOverlay(point, vec3_t(-0.5f, -0.5f, -0.5f), vec3_t(0.5f, 0.5f, 0.5f), ang_t(), 255, 0, 0, 150, 0.05f);
			const bool center = point == points.front();
			penetration::PenetrationInput_t in;

			in.m_damage = 1.f;
			in.m_damage_pen = 1.f;
			in.m_can_pen = pen;
			in.m_target = m_player;
			in.m_from = g_cl.m_local;
			in.m_pos = point;
			in.m_center = center;

			penetration::PenetrationOutput_t out;

			const bool did_hit = penetration::run(&in, &out);

			if (center && delay_shot > 0) {

				if (out.m_last_pen_count == 4 && out.m_damage < std::clamp(delay_shot == 1 ? dmg * 0.1f : dmg * 0.5f, 1.f, (float)hp)) {

					if (out.m_damage > 0.f) {
						this->m_hit = true;
						g_aimbot.m_found_hit = true;
					}

					break;
				}
				this->m_hit = true;
			}

			if (did_hit) {
				this->m_hit = true;

				if (out.m_damage > dmg * 0.25)
					g_aimbot.m_found_hit = true;
			}

			// we can hit p!
			if (did_hit && (out.m_damage >= dmg || out.m_damage >= hp)) {

				// fix head behind body situations.
				if (it.m_index == HITBOX_HEAD && out.m_hitgroup != HITGROUP_HEAD)
					continue;


				// this hitbox requires lethality to get selected, if that is the case.
				// we are done, stop now.
				if (out.m_damage >= hp && it.m_index > 2)
					done = true;

				// prefered hitbox, just stop now.&
				else if (m_prefer_body && out.m_hitgroup != HITGROUP_HEAD && it.m_index > 2)
					done = true;

				// this hitbox has normal selection, it needs to have more damage.
				// we did more damage.
				else if (out.m_damage > scan.m_damage || (out.m_damage >= scan.m_damage && prefer_center && center)) {

					// save new best data->
					scan.m_damage = out.m_damage;
					scan.m_pos = point;
					scan.m_hitbox = it.m_index;
					scan.m_hitgroup = out.m_hitgroup;

					if (center && (out.m_damage >= hp || prefer_center))
						break;
				}

				// we found a preferred / lethal hitbox.
				if (done) {
					// save new best data->
					scan.m_damage = out.m_damage;
					scan.m_pos = point;
					scan.m_hitbox = it.m_index;
					scan.m_hitgroup = out.m_hitgroup;
					break;
				}
			}
		}

		// ghetto break out of outer loop.
		if (done) {
			break;
		}
	}

	g_aimbot.m_backup[m_player->index() - 1].restore(m_player);

	// we found something that we can damage.
	// set out vars.
	if (scan.m_damage > 0.f) {
		aim = scan.m_pos;
		damage = scan.m_damage;
		hitgroup = scan.m_hitgroup;
		hitbox = scan.m_hitbox;

		return true;
	}

	return false;
}

void Aimbot::apply() {
	bool attack, attack2;

	// attack states.
	attack = (g_cl.m_cmd->m_buttons & IN_ATTACK);
	attack2 = (g_cl.m_weapon_id == REVOLVER && g_cl.m_cmd->m_buttons & IN_ATTACK2);

	// ensure we're attacking.
	if (attack || attack2) {

		// choke every shot.
		if (!g_menu.main.antiaim.fakelag_silent_aim.get())
			*g_cl.m_packet = true;
		else {

			*g_cl.m_packet = g_cl.m_lag >= 14;

			if (g_cl.m_lag >= 14) {
				StripAttack();
				return;
			}
		}

		if (m_target) {
			// make sure to aim at un-interpolated data->
			// do this so BacktrackEntity selects the exact record.
			if (m_record && !m_record->m_broke_lc)
				g_cl.m_cmd->m_tick = game::TIME_TO_TICKS(m_record->m_pred_time + g_cl.m_lerp);

			// set angles to target.
			g_cl.m_cmd->m_view_angles = m_angle;

			// if not silent aim, apply the viewangles.
			if (!g_menu.main.aimbot.silent.get())
				g_csgo.m_engine->SetViewAngles(m_angle);

			if (g_menu.main.aimbot.debugaim.get())
				g_visuals.DrawHitboxMatrix(m_record, g_menu.main.aimbot.debugaim_color.get(), 3.f, g_menu.main.aimbot.debugaim_mode.get());


			// store fired shot.
			g_shots.OnShotFire(m_target ? m_target : nullptr, m_target ? m_damage : -1.f, g_cl.m_weapon_info->m_bullets, m_target ? m_record : nullptr, m_target ? m_aim : vec3_t(0, 0, 0), m_hitbox, m_hitgroup);
		}

		// nospread.
		if (g_menu.main.aimbot.nospread.get())
			NoSpread();

		// norecoil.
		if (g_menu.main.aimbot.norecoil.get())
			g_cl.m_cmd->m_view_angles -= g_cl.m_local->m_aimPunchAngle() * g_csgo.weapon_recoil_scale->GetFloat();

		// set that we fired.
		g_cl.m_shot = true;
	}
}

void Aimbot::NoSpread() {
	bool    attack2;
	vec3_t  spread, forward, right, up, dir;

	// revolver state.
	attack2 = (g_cl.m_weapon_id == REVOLVER && (g_cl.m_cmd->m_buttons & IN_ATTACK2));

	// get spread.
	spread = g_cl.m_weapon->CalculateSpread(g_cl.m_cmd->m_random_seed, attack2);

	// compensate.
	g_cl.m_cmd->m_view_angles -= { -math::rad_to_deg(std::atan(spread.length_2d())), 0.f, math::rad_to_deg(std::atan2(spread.x, spread.y)) };
}

bool Aimbot::CanHitRecordHead(LagRecord* record) {

	if (!record->m_player)
		return false;

	const model_t* model = record->m_player->GetModel();
	if (!model)
		return false;

	studiohdr_t* hdr = g_csgo.m_model_info->GetStudioModel(model);
	if (!hdr)
		return false;

	mstudiohitboxset_t* set = hdr->GetHitboxSet(record->m_player->m_nHitboxSet());
	if (!set)
		return false;

	mstudiobbox_t* bbox = set->GetHitbox(HITBOX_HEAD);
	if (!bbox)
		return false;

	vec3_t p = (bbox->m_mins + bbox->m_maxs) / 2.f;
	math::VectorTransform(p, record->m_bones[bbox->m_bone], p);

	penetration::PenetrationInput_t in;

	in.m_damage = 1.f;
	in.m_damage_pen = 1.f;
	in.m_can_pen = true;
	in.m_target = record->m_player;
	in.m_from = g_cl.m_local;
	in.m_pos = p;
	in.m_center = false;

	penetration::PenetrationOutput_t out;

	record->cache();
	return penetration::run(&in, &out);
}