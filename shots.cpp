#include "includes.h"

Shots g_shots{ };

void Shots::OnShotFire(Player* target, float damage, int bullets, LagRecord* record, vec3_t aim_point, int hitbox, int hitgroup) {
	// we are not shooting manually.
	// and this is the first bullet, only do this once.
	if (target && record) {
		// setup new shot data.
		ShotRecord shot;
		shot.m_target = target;
		shot.m_record = record;
		shot.m_time = g_csgo.m_globals->m_realtime;
		shot.m_lat = g_cl.m_latency;
		shot.m_damage = damage;
		shot.m_pos = g_cl.m_shoot_pos;
		shot.m_impacted = false;
		shot.m_confirmed = false;
		shot.m_hurt = false;
		shot.m_invalid_record = record->valid();
		shot.m_range = g_cl.m_weapon_info->m_range;
		shot.m_aim_point = aim_point;
		shot.m_hitbox = hitbox;
		shot.m_hitgroup = hitgroup;
		shot.m_had_pred_error = g_csgo.m_globals->m_tick_count - g_cl.m_cmd->m_tick < -1;

		// increment total shots on this player.
		AimPlayer* data = &g_aimbot.m_players[target->index() - 1];
		player_info_t info;
		bool success = g_csgo.m_engine->GetPlayerInfo(target->index(), &info);

		if (data)
			++data->m_shots;

		bool broke_lc = shot.m_record->broke_lc() || shot.m_record->m_sim_time <= shot.m_record->m_old_sim_time;

		bool delay = g_menu.main.aimbot.prefer_accuracy.get(0) && shot.m_record->broke_lc();

		int hc = (int)std::round( g_aimbot.m_hit_chance );

		int vel_rounded2 = static_cast < int >( std::round( record->m_base_vel.length_2d( ) ) );
		int vel_rounded1 = static_cast < int >( std::round( record->m_anim_velocity.length_2d( ) ) );

		if (g_cl.m_userr == "dev") {
			if (success)
				// TODO: fix this; m_hitbox is being set in getbestaimposition and find. why it's not working? no fucking idea. 
				// fired shot at %s in the %s for %i damage | pb[%s], hit_c[%i], mode[%i], bt[%i], vel_2d[%i:%i], lag[%s:%s], lc[%s:%s], exp[%s]\n
				g_notify.add(tfm::format("[dbg] fired shot | ent: %s, hb: %s (%i dmg, %s, %i), mode: %i, bt: %i, vel: %i (%i, [%s:%s]), lag: %s, ext: %s | %s\n",
					info.m_name, 
					m_groups[shot.m_hitgroup], 
					shot.m_damage, 
					data->m_prefer_body,
					hc, 
					shot.m_record->m_mode,
					game::TIME_TO_TICKS(data->m_records.front().get()->m_sim_time - record->m_sim_time), 
					vel_rounded1, 
					vel_rounded2, 
					std::to_string(record->m_lag),
					std::to_string(record->m_anim_lag),
					std::to_string(record->m_broke_lc),
					std::to_string(record->m_extrapolated),
					std::to_string(broke_lc)));
		}

		// add to tracks.
		m_shots.push_front(shot);
	}

	// no need to keep an insane amount of shots.
	while (m_shots.size() > 128)
		m_shots.pop_back();
}

void Shots::OnImpact(IGameEvent* evt) {
	int        attacker;
	vec3_t     pos, dir, start, end;
	float      time;
	CGameTrace trace;

	// screw this.
	if (!evt || !g_cl.m_local)
		return;

	// get attacker, if its not us, screw it.
	attacker = g_csgo.m_engine->GetPlayerForUserID(evt->m_keys->FindKey(HASH("userid"))->GetInt());
	if (attacker != g_csgo.m_engine->GetLocalPlayer())
		return;

	// decode impact coordinates and convert to vec3.
	pos = {
		evt->m_keys->FindKey(HASH("x"))->GetFloat(),
		evt->m_keys->FindKey(HASH("y"))->GetFloat(),
		evt->m_keys->FindKey(HASH("z"))->GetFloat()
	};

	// get prediction time at this point.
	time = game::TICKS_TO_TIME(g_cl.m_local->m_nTickBase());

	// add to visual impacts if we have features that rely on it enabled.
	// todo - dex; need to match shots for this to have proper GetShootPosition, don't really care to do it anymore.
	if (g_menu.main.visuals.impact_beams.get())
		m_vis_impacts.push_back({ pos, g_cl.m_local->GetShootPosition(), g_cl.m_local->m_nTickBase() });

	// we did not take a shot yet.
	if (m_shots.empty())
		return;

	struct ShotMatch_t { float delta; ShotRecord* shot; };
	ShotMatch_t match;
	match.delta = std::numeric_limits< float >::max();
	match.shot = nullptr;

	// iterate all shots.
	for (auto& s : m_shots) {

		// this shot was already matched
		// with a 'bullet_impact' event.
		if (s.m_impacted)
			continue;

		// get the delta between the current time
		// and the predicted arrival time of the shot.
		float delta = g_csgo.m_globals->m_realtime - s.m_time;

		// store this shot as being the best for now.
		// NOTE: changed to <= instead of < cus usually last impact is the "best" one
		if (delta <= match.delta) {
			match.delta = delta;
			match.shot = &s;
		}
	}

	// no valid shotrecord was found.
	ShotRecord* shot = match.shot;
	if (!shot)
		return;

	// this shot was matched.
	shot->m_impacted = true;
	shot->m_impact_pos = pos;
}

void Shots::OnHurt(IGameEvent* evt) {
	int         attacker, victim, group, hp;
	float       time, damage;
	std::string name;

	if (!evt || !g_cl.m_local)
		return;

	attacker = g_csgo.m_engine->GetPlayerForUserID(evt->m_keys->FindKey(HASH("attacker"))->GetInt());
	victim = g_csgo.m_engine->GetPlayerForUserID(evt->m_keys->FindKey(HASH("userid"))->GetInt());

	// skip invalid player indexes.
	// should never happen? world entity could be attacker, or a nade that hits you.
	if (attacker < 1 || attacker > 64 || victim < 1 || victim > 64)
		return;

	// we were not the attacker or we hurt ourselves.
	else if (attacker != g_csgo.m_engine->GetLocalPlayer() || victim == g_csgo.m_engine->GetLocalPlayer())
		return;

	// get hitgroup.
	// players that get naded ( DMG_BLAST ) or stabbed seem to be put as HITGROUP_GENERIC.
	group = evt->m_keys->FindKey(HASH("hitgroup"))->GetInt();

	// invalid hitgroups ( note - dex; HITGROUP_GEAR isn't really invalid, seems to be set for hands and stuff? ).
	if (group == HITGROUP_GEAR)
		return;

	// get the player that was hurt.
	Player* target = g_csgo.m_entlist->GetClientEntity< Player* >(victim);
	if (!target)
		return;

	// get player info.
	player_info_t info;
	if (!g_csgo.m_engine->GetPlayerInfo(victim, &info))
		return;

	// get player name;
	name = std::string(info.m_name).substr(0, 24);

	// get damage reported by the server.
	damage = (float)evt->m_keys->FindKey(HASH("dmg_health"))->GetInt();

	// get remaining hp.
	hp = evt->m_keys->FindKey(HASH("health"))->GetInt();

	// setup family watermark thing
	if (group == HITGROUP_HEAD && damage >= 100.f)
		taps++;

	// setup headshot marker
	if (group == HITGROUP_HEAD)
		iHeadshot = true;
	else
		iHeadshot = false;

	// get prediction time at this point.
	time = game::TICKS_TO_TIME(g_cl.m_local->m_nTickBase());

	// hitmarker stuff lol
	g_visuals.m_hit_duration = 1.f; // 0.25
	g_visuals.m_hit_start = g_csgo.m_globals->m_curtime;
	g_visuals.m_hit_end = g_visuals.m_hit_start + g_visuals.m_hit_duration;

	// hitsound.
	if (g_menu.main.players.hitmarker_sound.get()) {
		g_csgo.m_sound->EmitAmbientSound(XOR("buttons/arena_switch_press_02.wav"), 1.f);
	}

	if (g_menu.main.misc.logdamagedealt.get()) {
		std::string out = tfm::format(XOR("hit %s in the %s for %i damage (%i health remaining)\n"), name, m_groups[group], (int)damage, hp);
		g_notify.add(out);
	}

	// print this shit.
	if (group == HITGROUP_GENERIC)
		return;

	// if we hit a player, mark vis impacts.
	if (!m_vis_impacts.empty()) {
		for (auto& i : m_vis_impacts) {
			if (i.m_tickbase == g_cl.m_local->m_nTickBase())
				i.m_hit_player = true;
		}
	}

	struct ShotMatch_t { float delta; ShotRecord* shot; };
	ShotMatch_t match;
	match.delta = std::numeric_limits< float >::max();
	match.shot = nullptr;

	// iterate all shots.
	for (auto& s : m_shots) {

		// this shot was already matched
		// with a 'player_hurt' event.
		if (s.m_hurt)
			continue;

		// get the delta between the current time
		// and the predicted arrival time of the shot.
		float delta = std::fabsf(g_csgo.m_globals->m_realtime - s.m_time);

		// store this shot as being the best for now.
		if (delta <= match.delta) {
			match.delta = delta;
			match.shot = &s;
		}
	}

	// no valid shotrecord was found.
	ShotRecord* shot = match.shot;
	if (!shot)
		return;

	// this shot was matched.
	shot->m_invalid_record = shot->m_record->valid();
	shot->m_hurt = true;
}

void Shots::OnWeaponFire(IGameEvent* evt) {
	int        attacker;

	// screw this.
	if (!evt || !g_cl.m_local)
		return;

	// get attacker, if its not us, screw it.
	attacker = g_csgo.m_engine->GetPlayerForUserID(evt->m_keys->FindKey(HASH("userid"))->GetInt());
	if (attacker != g_csgo.m_engine->GetLocalPlayer())
		return;

	struct ShotMatch_t { float delta; ShotRecord* shot; };
	ShotMatch_t match;
	match.delta = std::numeric_limits< float >::max();
	match.shot = nullptr;

	// iterate all shots.
	for (auto& s : m_shots) {

		// this shot was already matched
		// with a 'weapon_fire' event.
		if (s.m_confirmed)
			continue;

		// get the delta between the current time
		// and the predicted arrival time of the shot.
		float delta = g_csgo.m_globals->m_realtime - s.m_time;

		// store this shot as being the best for now.
		if (delta <= match.delta) {
			match.delta = delta;
			match.shot = &s;
		}
	}

	// no valid shotrecord was found.
	ShotRecord* shot = match.shot;
	if (!shot)
		return;

	// this shot was matched.
	shot->m_confirmed = true;
}

void Shots::OnShotMiss(ShotRecord& shot) {
	vec3_t     pos, dir, start, end;
	CGameTrace trace;

	// shots we fire manually won't have a record.
	if (!shot.m_record)
		return;

	// not in nospread mode, see if the shot missed due to spread.
	Player* target = shot.m_target;
	if (!target)
		return;

	// not gonna bother anymore.
	if (g_menu.main.aimbot.debuglog.get()) {
		if (!target->alive()) {
			//if( g_menu.main.misc.notifications.get( 6 ) )
			g_cl.print("missed shot due to player death\n");
			return;
		}
	}

	AimPlayer* data = &g_aimbot.m_players[target->index() - 1];
	if (!data)
		return;

	// this record was deleted already.
	if (g_menu.main.aimbot.debuglog.get()) {
		if (!shot.m_record->m_bones) {
			g_notify.add(XOR("missed shot due to invalid record\n"), Color(255, 0, 0, 255));
			return;
		}
	}

	// we are going to alter this player.
	// store all his og data.
	g_aimbot.m_backup[ target->index( ) ].store(target);

	// write historical matrix of the time that we shot
	// into the games bone cache, so we can trace against it.
	shot.m_record->cache();

	// start position of trace is where we took the shot.
	start = shot.m_pos;

	// where our shot landed at.
	pos = shot.m_impact_pos;

	// the impact pos contains the spread from the server
	// which is generated with the server seed, so this is where the bullet
	// actually went, compute the direction of this from where the shot landed
	// and from where we actually took the shot.
	dir = (pos - start).normalized();

	// get end pos by extending direction forward.
	end = start + (dir * shot.m_range);

	// intersect our historical matrix with the path the shot took.
	g_csgo.m_engine_trace->ClipRayToEntity(Ray(start, end), CS_MASK_SHOOT | CONTENTS_HITBOX, target, &trace);

	if (g_menu.main.aimbot.debuglog.get()) {
		// we did not hit jackshit, or someone else.
		if (trace.m_entity == target || g_menu.main.aimbot.nospread.get()) {
			size_t mode = shot.m_record->m_mode;
			int curr_mode_miss = 0;

			// if we miss a shot on body update.
			// we can chose to stop shooting at them.
			if (mode == Resolver::Modes::RESOLVE_DATA) {
				++data->m_stand_move_idx;
				curr_mode_miss = data->m_stand_move_idx;
			}
			else if (mode == Resolver::Modes::RESOLVE_NO_DATA) {
				++data->m_stand_no_move_idx;
				curr_mode_miss = data->m_stand_no_move_idx;
			}
			else if (mode == Resolver::Modes::RESOLVE_LBY) {
				++data->m_body_idx;
				curr_mode_miss = data->m_body_idx;
			}
			else if (mode == Resolver::Modes::RESOLVE_AIR) {
				++data->m_air_idx;
				curr_mode_miss = data->m_air_idx;
			}
			else if (mode == Resolver::Modes::RESOLVE_LBY_PRED) {

				// increment lby pred miss
				++data->m_body_pred_idx;
				curr_mode_miss = data->m_body_pred_idx;

				// if we mispredict it means hes not at his lby
				// in that case, blacklist lby 
				++data->m_body_idx;
			}

			// we will not shoot this shitty mode twice
			if (shot.m_record->m_resolver_mode == "M:INVERTFS")
				data->m_missed_invertfs = true;

			if (std::abs(math::AngleDiff(shot.m_record->m_back, shot.m_record->m_eye_angles.y)) <= 10.f)
				data->m_missed_back = true;

			// if mode isnt lby nor walk
			if (mode != Resolver::Modes::RESOLVE_LBY
				&& mode != Resolver::Modes::RESOLVE_WALK) {

				const float diff = std::abs(math::AngleDiff(shot.m_record->m_body, shot.m_record->m_eye_angles.y));

				// but delta is really close
				// then lets pretend we missed it
				// so we dont shoot the same angle twice
				if (diff <= 10.f)
					++data->m_body_idx;
			}

			++data->m_missed_shots;

			if (mode == Resolver::Modes::RESOLVE_WALK)
				g_notify.add(XOR("missed shot due to lag compensation\n"));
			else if (mode != Resolver::Modes::RESOLVE_NONE)
				g_notify.add(XOR("missed shot due to fake angles\n"));
		}
		else
			g_notify.add(XOR("missed shot due to spread\n"));
	}

	// restore player to his original state.
	g_aimbot.m_backup[ target->index( ) ].restore(target);
}

void Shots::Think() {


	if (!g_cl.m_processing || m_shots.empty()) {
		// we're dead, we won't need this data anymore.
		if (!m_shots.empty()) {
			m_shots.clear();
			g_cl.print("missed shot due to death\n");
		}

		// we don't handle shots if we're dead or if there are none to handle.
		return;
	}

	// iterate all shots.
	for (auto it = m_shots.begin(); it != m_shots.end(); ) {
		// too much time has passed, we don't need this anymore.
		if (it->m_time + 1.f < g_csgo.m_globals->m_realtime) {
			if (!it->m_impacted && it->m_confirmed && it->m_target && it->m_target->alive())
				g_cl.print("missed shot due to unregistered shot\n");

			// remove it.
			it = m_shots.erase(it);
		}
		else
			it = next(it);
	}

	// iterate all shots.
	for (auto it = m_shots.begin(); it != m_shots.end(); ) {
		// our shot impacted, and it was confirmed, but we didn't damage anyone. we missed.
		if (it->m_impacted && it->m_confirmed && !it->m_hurt) {
			// handle the shot.
			OnShotMiss(*it);

			// since we've handled this shot, we won't need it anymore.
			it = m_shots.erase(it);
		}
		else
			it = next(it);
	}
}