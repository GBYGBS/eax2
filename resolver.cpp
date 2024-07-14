#include "includes.h"

Resolver g_resolver{};

float Resolver::AntiFreestand(Player* player, LagRecord* record, vec3_t start_, vec3_t end, bool include_base, float base_yaw, float delta) {
	AimPlayer* data = &g_aimbot.m_players[player->index() - 1];

	// Constants
	constexpr float STEP{ 4.f };
	constexpr float RANGE{ 32.f };

	// Construct vector of angles to test
	std::vector<AdaptiveAngle> angles;
	angles.reserve(3);  // Reserve space to avoid reallocation

	angles.emplace_back(base_yaw + delta);
	angles.emplace_back(base_yaw - delta);

	if (include_base) {
		angles.emplace_back(base_yaw);
	}

	// Start the trace at the enemy shoot pos
	vec3_t start = start_;
	vec3_t shoot_pos = end;

	bool valid{ false };

	// Iterate over the vector of angles
	for (auto& angle : angles) {
		// Compute the 'rough' estimation of where our head will be
		vec3_t end{
			shoot_pos.x + std::cos(math::deg_to_rad(angle.m_yaw)) * RANGE,
			shoot_pos.y + std::sin(math::deg_to_rad(angle.m_yaw)) * RANGE,
			shoot_pos.z
		};

		// Compute the direction
		vec3_t dir = end - start;
		float len = dir.normalize();

		// Ensure len is positive
		if (len <= 0.f)
			continue;

		// Step through the total distance, 4 units per step
		for (float i = 0.f; i < len; i += STEP) {
			vec3_t point = start + (dir * i);
			int contents = g_csgo.m_engine_trace->GetPointContents(point, MASK_SHOT_HULL);

			// If the point is obstructed, accumulate the distance
			if (contents & MASK_SHOT_HULL) {
				float mult = 1.f;
				if (i > (len * 0.5f))
					mult = 1.25f;
				if (i > (len * 0.75f))
					mult = 1.25f;
				if (i > (len * 0.9f))
					mult = 2.f;

				angle.m_dist += (STEP * mult);
				valid = true;
			}
		}
	}

	if (!valid)
		return base_yaw;

	// Sort the angles based on the distance
	std::sort(angles.begin(), angles.end(), [](const AdaptiveAngle& a, const AdaptiveAngle& b) {
		return a.m_dist > b.m_dist;
		});

	// The best angle should be at the front now
	return angles.front().m_yaw;
}

LagRecord* Resolver::FindIdealRecord(AimPlayer* data) {
	LagRecord* first_valid, * current, * first_flick;

	if (data->m_records.empty())
		return nullptr;

	first_flick = nullptr;
	first_valid = nullptr;


	LagRecord* front = data->m_records.front().get();

	if (front && (front->broke_lc() || front->m_sim_time < front->m_old_sim_time)) {


		if (front->valid())
			return front;

		return nullptr;
	}

	// iterate records.
	for (const auto& it : data->m_records) {
		if (it->dormant() || it->immune() || !it->valid())
			continue;

		// get current record.
		current = it.get();

		// ok, stop lagcompensating here
		if (current->broke_lc())
			break;

		// first record that was valid, store it for later.
		if (!first_valid)
			first_valid = current;

		if (!first_flick && (it->m_mode == Modes::RESOLVE_LBY || it->m_mode == Modes::RESOLVE_LBY_PRED))
			first_flick = current;

		// try to find a record with a shot, lby update, walking or no anti-aim.
		if (it->m_mode == Modes::RESOLVE_WALK && it->m_ground_for_two_ticks) {
			// only shoot at ideal record if we can hit head (usually baim is faster if we shoot at front)
			if (it->m_origin.dist_to(data->m_records.front()->m_origin) <= 0.1f || g_aimbot.CanHitRecordHead(current))
				return current;
		}
	}

	if (first_flick)
		return first_flick;

	// none found above, return the first valid record if possible.
	return (first_valid) ? first_valid : nullptr;
}

LagRecord* Resolver::FindLastRecord(AimPlayer* data) {
	if (data->m_records.empty())
		return nullptr;

	LagRecord* front = data->m_records.front().get();

	if (front && (front->broke_lc() || front->m_sim_time < front->m_old_sim_time))
		return nullptr;

	bool found_last = false;

	// Iterate records in reverse
	for (auto it = data->m_records.crbegin(); it != data->m_records.crend(); ++it) {
		LagRecord* current = it->get();

		if (current->broke_lc())
			break;

		if (current->valid() && !current->immune() && !current->dormant()) {
			if (found_last)
				return current;

			found_last = true;
		}
	}

	return nullptr;
}

void Resolver::OnBodyUpdate(Player* player, float value) {
	if (!player || !player->alive() || value < 0.f || value > 179.f)
		return;

	int playerIndex = player->index();
	if (playerIndex <= 0 || playerIndex > g_csgo.m_globals->m_max_clients) {
		return;  // Ensure player index is within valid range
	}

	AimPlayer* data = &g_aimbot.m_players[playerIndex - 1];
	if (!data) {
		return;  // Ensure AimPlayer data is valid
	}

	// Set data
	data->m_old_body = data->m_body;
	data->m_body = value;
	data->m_upd_time = g_csgo.m_globals->m_curtime;  // Store the update time
}

float Resolver::GetAwayAngle(LagRecord* record) {
	int nearest_idx = GetNearestEntity(record->m_player, record);
	Player* nearest = static_cast<Player*>(g_csgo.m_entlist->GetClientEntity(nearest_idx));

	if (!nearest)
		return 0.f;

	ang_t away;
	math::VectorAngles(nearest->m_vecOrigin() - record->m_pred_origin, away);
	return away.y;
}

void Resolver::MatchShot(AimPlayer* data, LagRecord* record) {
	Weapon* wpn = data->m_player->GetActiveWeapon();
	if (!wpn)
		return;

	WeaponInfo* wpn_data = wpn->GetWpnData();
	if (!wpn_data)
		return;

	if (wpn_data->m_weapon_type != WEAPONTYPE_GRENADE && (wpn_data->m_weapon_type > WEAPONTYPE_SHOTGUN || wpn_data->m_weapon_type <= WEAPONTYPE_UNKNOWN))
		return;

	const auto shot_time = wpn->m_fLastShotTime();
	const auto shot_tick = game::TIME_TO_TICKS(shot_time);

	if (shot_tick == game::TIME_TO_TICKS(record->m_sim_time) && record->m_lag <= 2) {
		record->m_shot_type = 2;
	}
	else {
		bool should_correct_pitch = false;

		if (shot_tick == game::TIME_TO_TICKS(record->m_anim_time)) {
			record->m_shot_type = 1;
			should_correct_pitch = true;
		}
		else if (shot_tick >= game::TIME_TO_TICKS(record->m_anim_time)) {
			if (shot_tick <= game::TIME_TO_TICKS(record->m_sim_time))
				should_correct_pitch = true;
		}

		if (should_correct_pitch) {
			float valid_pitch = 89.f;

			for (const auto& it : data->m_records) {
				if (it.get() == record || it->dormant() || it->immune())
					continue;

				if (it->m_shot_type <= 0) {
					valid_pitch = it->m_eye_angles.x;
					break;
				}
			}

			record->m_eye_angles.x = valid_pitch;
		}
	}

	if (record->m_shot_type > 0)
		record->m_resolver_mode = "SHOT";
}

void Resolver::SetMode(LagRecord* record) {
	// Determine the resolver mode based on player's movement and flags.
	float speed = record->m_velocity.length_2d();
	const int flags = record->m_broke_lc ? record->m_pred_flags : record->m_player->m_fFlags();

	if (flags & FL_ONGROUND) {
		if (speed <= 35.f && g_input.GetKeyState(g_menu.main.aimbot.override.get())) {
			record->m_mode = Modes::RESOLVE_OVERRIDE;
		}
		else if (speed <= 0.1f || record->m_fake_walk) {
			record->m_mode = Modes::RESOLVE_STAND;
		}
		else {
			record->m_mode = Modes::RESOLVE_WALK;
		}
	}
	else {
		record->m_mode = Modes::RESOLVE_AIR;
	}
}

bool Resolver::IsSideways(float angle, LagRecord* record) {
	ang_t away;
	math::VectorAngles(g_cl.m_shoot_pos - record->m_pred_origin, away);
	const float diff = math::AngleDiff(away.y, angle);
	return diff > 45.f && diff < 135.f;
}

void Resolver::ResolveAngles(Player* player, LagRecord* record) {
	if (!player || !record || !record->m_player->alive())
		return;

	if (record->m_weapon) {
		WeaponInfo* wpn_data = record->m_weapon->GetWpnData();
		if (wpn_data && wpn_data->m_weapon_type == WEAPONTYPE_GRENADE) {
			if (record->m_weapon->m_bPinPulled() && record->m_weapon->m_fThrowTime() > 0.0f) {
				record->m_resolver_mode = "PIN";
				return;
			}
		}
	}

	if (player->m_MoveType() == MOVETYPE_LADDER || player->m_MoveType() == MOVETYPE_NOCLIP) {
		record->m_resolver_mode = "LADDER";
		return;
	}

	AimPlayer* data = &g_aimbot.m_players[player->index() - 1];
	MatchShot(data, record);

	if (data->m_last_stored_body == FLT_MIN) {
		data->m_last_stored_body = record->m_body;
	}

	if (record->m_velocity.length_2d() > 0.1f && (record->m_flags & FL_ONGROUND)) {
		data->m_has_ever_updated = false;
		data->m_last_stored_body = record->m_body;
		data->m_change_stored = 0;
	}
	else if (std::fabs(math::AngleDiff(data->m_last_stored_body, record->m_body)) > 1.f && record->m_shot_type <= 0) {
		data->m_has_ever_updated = true;
		data->m_last_stored_body = record->m_body;
		++data->m_change_stored;
	}

	if (data->m_records.size() >= 2 && record->m_shot_type <= 0) {
		LagRecord* previous = data->m_records[1].get();
		const float lby_delta = math::AngleDiff(record->m_body, previous->m_body);

		if (std::fabs(lby_delta) > 0.5f && !previous->m_dormant) {
			data->m_body_timer = FLT_MIN;
			data->m_body_updated_idk = 0;

			if (data->m_has_updated) {
				if (std::fabs(lby_delta) <= 155.f && std::fabs(lby_delta) > 25.f) {
					if (record->m_flags & FL_ONGROUND) {
						if (std::fabs(record->m_anim_time - data->m_upd_time) < 1.5f) {
							++data->m_update_count;
						}
						data->m_upd_time = record->m_anim_time;
					}
				}
				else {
					data->m_has_updated = 0;
					data->m_update_captured = 0;
				}
			}
			else if (std::fabs(lby_delta) > 25.f) {
				if (record->m_flags & FL_ONGROUND) {
					if (std::fabs(record->m_anim_time - data->m_upd_time) < 1.5f) {
						++data->m_update_count;
					}
					data->m_upd_time = record->m_anim_time;
				}
			}
		}
	}

	record->m_resolver_mode = "NONE";
	SetMode(record);

	if (record->m_mode != Modes::RESOLVE_WALK && record->m_shot_type <= 0) {
		LagRecord* previous = data->m_records.size() >= 2 ? data->m_records[1].get() : nullptr;

		if (previous && !previous->dormant()) {
			const float yaw_diff = math::AngleDiff(previous->m_eye_angles.y, record->m_eye_angles.y);
			const float body_diff = math::AngleDiff(record->m_body, record->m_eye_angles.y);
			const float eye_diff = record->m_eye_angles.x - previous->m_eye_angles.x;

			if (std::abs(eye_diff) <= 35.f && std::abs(record->m_eye_angles.x) <= 45.f && std::abs(yaw_diff) <= 45.f) {
				record->m_resolver_mode = "PITCH 0";
				return;
			}
		}
	}

	switch (record->m_mode) {
	case Modes::RESOLVE_WALK:
		ResolveWalk(data, record);
		break;
	case Modes::RESOLVE_STAND:
		ResolveStand(data, record, player);
		break;
	case Modes::RESOLVE_AIR:
		ResolveAir(data, record);
		break;
	case Modes::RESOLVE_OVERRIDE:
		ResolveOverride(data, record, record->m_player);
		break;
	default:
		break;
	}

	if (data->m_old_stand_move_idx != data->m_stand_move_idx || data->m_old_stand_no_move_idx != data->m_stand_no_move_idx) {
		data->m_old_stand_move_idx = data->m_stand_move_idx;
		data->m_old_stand_no_move_idx = data->m_stand_no_move_idx;

		if (auto animstate = player->m_PlayerAnimState(); animstate != nullptr) {
			animstate->m_foot_yaw = record->m_eye_angles.y;
			player->SetAbsAngles(ang_t{ 0, animstate->m_foot_yaw, 0 });
		}
	}

	math::NormalizeAngle(record->m_eye_angles.y);
}

void Resolver::ResolveAir(AimPlayer* data, LagRecord* record) {
	
}


void Resolver::ResolveWalk(AimPlayer* data, LagRecord* record) {
	// apply lby to eyeangles.
	record->m_eye_angles.y = record->m_body;

	// reset stand and body index.

	data->m_body_timer = record->m_anim_time + 0.22f;
	data->m_body_updated_idk = 0;
	data->m_update_captured = 0;
	data->m_has_updated = 0;
	data->m_last_body = FLT_MIN;
	data->m_overlap_offset = 0.f;

	const float speed_2d = record->m_velocity.length_2d();

	if (speed_2d > record->m_max_speed * 0.34f) {
		data->m_update_count = 0;
		data->m_upd_time = FLT_MIN;
		data->m_body_pred_idx
			= data->m_body_idx
			= data->m_old_stand_move_idx
			= data->m_old_stand_no_move_idx
			= data->m_stand_move_idx
			= data->m_stand_no_move_idx = 0;
		data->m_missed_back = data->m_missed_invertfs = false;
	}

	// copy the last record that this player was walking
	// we need it later on because it gives us crucial data.
	// copy move data over
	if (speed_2d > 25.f)
		data->m_walk_record.m_body = record->m_body;

	data->m_walk_record.m_origin = record->m_origin;
	data->m_walk_record.m_anim_time = record->m_anim_time;
	data->m_walk_record.m_sim_time = record->m_sim_time;

	record->m_resolver_mode = "WALK";
}


int Resolver::GetNearestEntity(Player* target, LagRecord* record) {

	// best data
	int idx = g_csgo.m_engine->GetLocalPlayer();
	float best_distance = g_cl.m_local && g_cl.m_processing ? g_cl.m_local->m_vecOrigin().dist_to(record->m_pred_origin) : FLT_MAX;

	// cur data
	Player* curr_player = nullptr;
	vec3_t  curr_origin{ };
	float   curr_dist = 0.f;
	AimPlayer* data = nullptr;

	for (int i{ 1 }; i <= g_csgo.m_globals->m_max_clients; ++i) {
		curr_player = g_csgo.m_entlist->GetClientEntity< Player* >(i);

		if (!curr_player
			|| !curr_player->IsPlayer()
			|| curr_player->index() > 64
			|| curr_player->index() <= 0
			|| !curr_player->enemy(target)
			|| curr_player->dormant()
			|| !curr_player->alive()
			|| curr_player == target)
			continue;

		curr_origin = curr_player->m_vecOrigin();
		curr_dist = record->m_pred_origin.dist_to(curr_origin);

		if (curr_dist < best_distance) {
			idx = i;
			best_distance = curr_dist;
		}
	}

	return idx;
}

bool Resolver::IsYawSideways(Player* entity, float yaw)
{
	auto local_player = g_cl.m_local;
	if (!local_player)
		return false;

	const auto at_target_yaw = math::CalcAngle(local_player->m_vecOrigin(), entity->m_vecOrigin()).y;
	const float delta = fabs(math::AngleDiff(at_target_yaw, yaw));

	return delta > 35.f && delta < 145.f;
}

void Resolver::ResolveStand(AimPlayer* data, LagRecord* record, Player* player)
{
	// get predicted away angle for the player.
	float away = GetAwayAngle(record);

	// pointer for easy access.
	LagRecord* move = &data->m_walk_record;
	LagRecord* previous = nullptr;

	if (data->m_records.size() > 1)
		previous = data->m_records[1].get();

	// we have a valid moving record.
	if (move->m_sim_time > 0.f)
	{
		vec3_t delta = move->m_origin - record->m_origin;

		// check if moving record is close.
		if (delta.length() <= 128.f && !record->m_fake_walk)
		{
			// indicate that we are using the moving lby.
			data->m_moved = true;
		}
	}

	if (record->m_fake_walk)
		data->is_last_moving_lby_valid = false;

	if (data->m_has_body_updated && (record->m_anim_time >= data->m_body_update) && data->m_body_idx <= 3)
	{
		record->m_eye_angles.y = record->m_body;
		data->m_body_update = record->m_anim_time + 1.1f;
		data->m_lbyticks++;
		data->m_flick_body = record->m_body;
		record->m_mode = Modes::RESOLVE_BODY_PRED;
		return;
	}

	bool above_120 = record->m_player->GetSequenceActivity(record->m_layers[3].m_sequence) == 979;

	if (data->m_lbyticks == 0 && data->m_lby_index <= 0 && !above_120 && fabsf(math::AngleDiff(data->m_old_body, record->m_body)) <= 25.f)
	{
		record->m_mode = Modes::RESOLVE_LBY;
		record->m_eye_angles.y = record->m_body;
		return;
	}

	if (!data->m_moved)
	{
		record->m_mode = Modes::RESOLVE_STAND3;

		switch (data->m_stand_index3 % 7)
		{
		case 0:
			if (fabsf(math::AngleDiff(record->m_body, data->m_flick_body)) <= 35.f)
				record->m_eye_angles.y = record->m_body;
			else
				record->m_eye_angles.y = away + 180.f;
			break;
		case 1:
			record->m_eye_angles.y = data->m_body;
			break;
		case 2:
			record->m_eye_angles.y = away - 70.f;
			break;
		case 3:
			record->m_eye_angles.y = away + 70.f;
			break;
		case 4:
			record->m_eye_angles.y = away + 120.f;
			break;
		case 5:
			record->m_eye_angles.y = away - 120.f;
			break;
		case 6:
			record->m_eye_angles.y = away;
			break;
		}
	}
	else
	{
		const float flMoveDelta = fabsf(math::AngleDiff(move->m_body, record->m_body));

		bool is_sideways = IsYawSideways(player, move->m_body);
		bool is_backwards = !is_sideways;
		ang_t edge;
		if (data->m_sidelast_index < 1 && IsYawSideways(player, move->m_body) && flMoveDelta < 12.5f && (data->is_last_moving_lby_valid && fabsf(math::AngleDiff(record->m_body, data->m_flick_body)) < 12.5f))
		{
			record->m_mode = Modes::RESOLVE_SIDE_LASTMOVE;
			record->m_eye_angles.y = move->m_body;
		}
		else if (data->m_reversefs_index < 1 && (is_sideways || (is_sideways && fabsf(math::AngleDiff(data->m_anti_fs_angle, data->m_flick_body)) <= 90.f)) && data->freestand_data)
		{
			record->m_mode = Modes::RESOLVE_REVERSEFS;
			record->m_eye_angles.y = data->m_anti_fs_angle;
		}
		else if (data->m_lastmove_index < 1 && flMoveDelta < 15.f && (data->is_last_moving_lby_valid && fabsf(math::AngleDiff(record->m_body, data->m_flick_body)) < 15.f))
		{
			record->m_mode = Modes::RESOLVE_LASTMOVE;
			record->m_eye_angles.y = move->m_body;
		}
		else if (data->m_back_index < 1 && (is_backwards || !data->freestand_data || (is_backwards && fabsf(math::AngleDiff(record->m_body, data->m_flick_body)) <= 35.f)))
		{
			record->m_mode = Modes::RESOLVE_BACK;
			record->m_eye_angles.y = away + 180.f;
		}
		else if (data->m_lastmove_index >= 1 || data->m_back_index >= 1 && is_backwards)
		{
			record->m_mode = Modes::RESOLVE_STAND2;
			switch (data->m_stand_index2 % 3)
			{
			case 0:
				record->m_eye_angles.y = data->m_flick_body;
				break;
			case 1:
				record->m_eye_angles.y = away + 120.f;
				break;
			case 2:
				record->m_eye_angles.y = away - 120.f;
				break;
			}
		}
		else if (data->m_reversefs_index >= 1 || data->m_sidelast_index >= 1 && is_sideways)
		{
			record->m_mode = Modes::RESOLVE_STAND1;
			switch (data->m_stand_index1 % 8)
			{
			case 0:
				record->m_eye_angles.y = -data->m_anti_fs_angle;
				break;
			case 1:
				record->m_eye_angles.y = +data->m_anti_fs_angle;
				break;
			case 2:
				record->m_eye_angles.y = away;
				break;
			case 3:
				record->m_eye_angles.y = away + 180.f;
				break;
			case 4:
				record->m_eye_angles.y = away + 90.f;
				break;
			case 5:
				record->m_eye_angles.y = away - 90.f;
				break;
			case 6:
				record->m_eye_angles.y = away + 110.f;
				break;
			case 7:
				record->m_eye_angles.y = away - 110.f;
				break;
			default:
				break;
			}
		}
	}
}

void Resolver::ResolveOverride(AimPlayer* data, LagRecord* record, Player* player) {
	// get predicted away angle for the player.
	float away = GetAwayAngle(record);

	C_AnimationLayer* curr = &record->m_layers[3];
	int act = data->m_player->GetSequenceActivity(curr->m_sequence);


	record->m_resolver_mode = "OVERRIDE";
	ang_t                          viewangles;
	g_csgo.m_engine->GetViewAngles(viewangles);

	//auto yaw = math::clamp (g_cl.m_local->GetAbsOrigin(), Player->origin()).y;
	const float at_target_yaw = math::CalcAngle(g_cl.m_local->m_vecOrigin(), player->m_vecOrigin()).y;
	const float dist = math::NormalizedAngle(viewangles.y - at_target_yaw);

	float brute = 0.f;

	if (std::abs(dist) <= 1.f) {
		brute = at_target_yaw;
		record->m_resolver_mode += ":BACK";
	}
	else if (dist > 0) {
		brute = at_target_yaw + 90.f;
		record->m_resolver_mode += ":RIGHT";
	}
	else {
		brute = at_target_yaw - 90.f;
		record->m_resolver_mode += ":LEFT";
	}

	record->m_eye_angles.y = brute;


}

#pragma optimize( "", on )