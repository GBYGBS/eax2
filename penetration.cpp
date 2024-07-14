#include "includes.h"


__forceinline auto DotProduct(const vec3_t& a, const vec3_t& b) -> float {
	return (a[0] * b[0] + a[1] * b[1] + a[2] * b[2]);
}


inline float DistanceToRay(const vec3_t& pos, const vec3_t& rayStart, const vec3_t& rayEnd, float* along = NULL, vec3_t* pointOnRay = NULL)
{
	vec3_t to = pos - rayStart;
	vec3_t dir = rayEnd - rayStart;
	float length = dir.normalize_in_place();

	float rangeAlong = DotProduct(dir, to);
	if (along)
	{
		*along = rangeAlong;
	}

	float range;

	if (rangeAlong < 0.0f)
	{
		// off start point
		range = -(pos - rayStart).length();

		if (pointOnRay)
		{
			*pointOnRay = rayStart;
		}
	}
	else if (rangeAlong > length)
	{
		// off end point
		range = -(pos - rayEnd).length();

		if (pointOnRay)
		{
			*pointOnRay = rayEnd;
		}
	}
	else // within ray bounds
	{
		vec3_t onRay = rayStart + rangeAlong * dir;
		range = (pos - onRay).length();

		if (pointOnRay)
		{
			*pointOnRay = onRay;
		}
	}

	return range;
}


bool is_armored(Player* player, int hit_group) {
	const bool has_helmet = player->m_bHasHelmet();
	const bool has_heavy_armor = player->m_bHasHeavyArmor();
	const float armor_val = player->m_ArmorValue();

	if (armor_val > 0.f) {
		switch (hit_group) {
		case 2:
		case 3:
		case 4:
		case 5:
			return true;
			break;
		case 1:
			return has_helmet || has_heavy_armor;
			break;
		default:
			return has_heavy_armor;
			break;
		}
	}

	return false;
}

float penetration::scale(Player* player, float damage, float armor_ratio, int hitgroup) {
	bool  has_heavy_armor{ false };
	int   armor;
	float heavy_ratio, ratio, new_damage;

	// check if the player has heavy armor, this is only really used in operation stuff.
	has_heavy_armor = player->m_bHasHeavyArmor(); //crash
	const int armor_val = player->m_ArmorValue();

	// scale damage based on hitgroup.
	switch (hitgroup) {
	case HITGROUP_HEAD:
		damage *= 4.f;

		if (has_heavy_armor)
			damage *= 0.5f;

		break;

	case HITGROUP_STOMACH:
		damage *= 1.25f;
		break;

	case HITGROUP_LEFTLEG:
	case HITGROUP_RIGHTLEG:
		damage *= 0.75f;
		break;

	default:
		break;
	}

	// grab amount of player armor.
	armor = player->m_ArmorValue();

	// check if the ent is armored and scale damage based on armor.
	if (is_armored(player, hitgroup)) {
		float armor_scale = 1.f;
		float armor_bonus_ratio = 0.5f;
		float armor_ratio_calced = armor_ratio * 0.5f;
		float dmg_to_health = 0.f;

		if (has_heavy_armor) {
			armor_ratio_calced = armor_ratio * 0.25f;
			armor_bonus_ratio = 0.33f;
			armor_scale = 0.33f;

			dmg_to_health = (damage * armor_ratio_calced) * 0.85f;
		}
		else
			dmg_to_health = damage * armor_ratio_calced;

		float dmg_to_armor = (damage - dmg_to_health) * (armor_scale * armor_bonus_ratio);

		if (dmg_to_armor > armor_val)
			dmg_to_health = damage - (armor_val / armor_bonus_ratio);

		damage = dmg_to_health;
	}

	return std::floor(damage);
}


void clip_trace_to_player(
	const vec3_t& src, const vec3_t& dst, CGameTrace& trace,
	Player* const player
)
{
	if (!player || !player->networkable()
		|| player->dormant() || !player->alive())
		return;

	// get bounding box
	const vec3_t mins = player->m_vecMins();
	const vec3_t maxs = player->m_vecMaxs();
	const vec3_t center = (maxs + mins) / 2.f;

	// calculate world space center
	const vec3_t vec_position = center + player->m_vecOrigin();

	const vec3_t vec_to = vec_position - src;

	vec3_t vec_direction = (dst - src);
	const float length = vec_direction.normalize();

	const float range_along = vec_direction.dot(vec_to);
	float range{ };

	// calculate distance to ray
	if (range_along < 0.0f)
		range = -vec_to.length(); // off start point
	else if (range_along > length)
		range = -(vec_position - dst).length(); // off end point
	else
		range = (vec_position - (vec_direction * range_along + src)).length();	// within ray bounds

	if (range < 0.f || range > 60.f)
		return;

	CGameTrace plr_tr;
	g_csgo.m_engine_trace->ClipRayToEntity(Ray{ src, dst }, CS_MASK_SHOOT | CONTENTS_HITBOX, player, &plr_tr);

	if (plr_tr.m_fraction > trace.m_fraction)
		plr_tr = trace;

}


void UTIL_ClipTraceToPlayer(const vec3_t& vecAbsStart, const vec3_t& vecAbsEnd, unsigned int mask, ITraceFilter* filter, CGameTrace* tr, Player* player)
{
	CGameTrace playerTrace;

	const float maxRange = 60.0f;

	if (!player || !player->alive())
		return;

	if (player->dormant())
		return;

	float range = DistanceToRay(player->WorldSpaceCenter(), vecAbsStart, vecAbsEnd);

	if (range < 0.0f || range > maxRange)
		return

		g_csgo.m_engine_trace->ClipRayToEntity(Ray(vecAbsStart, vecAbsEnd), mask | CONTENTS_HITBOX, player, &playerTrace);

	if (playerTrace.m_fraction < tr->m_fraction)
		*tr = playerTrace;
}




void TraceLine(const vec3_t& start, const vec3_t& end, uint32_t mask, ITraceFilter* ignore, CGameTrace* ptr) {
	Ray ray{ start, end };
	g_csgo.m_engine_trace->TraceRay(ray, mask, ignore, ptr);
}

bool CGameTrace::DidHitWorld() {
	if (!m_entity)
		return false;

	return m_entity == g_csgo.m_entlist->GetClientEntity(0);
}

bool CGameTrace::DidHitNonWorldEntity() {
	return m_entity != nullptr && !DidHitWorld();
}

bool penetration::TraceToExit(const vec3_t& start, const vec3_t& dir, vec3_t& out, CGameTrace* enter_trace, CGameTrace* exit_trace) {
	static CTraceFilterSimple_game filter{};

	float  dist{};
	vec3_t new_end;
	int    contents, first_contents{};

	// max pen distance is 90 units.
	do {
		// step forward a bit.
		dist += 4.f;

		// set out pos.
		out = start + (dir * dist);

		if (!first_contents)
			first_contents = g_csgo.m_engine_trace->GetPointContents(out, CS_MASK_SHOOT | CONTENTS_HITBOX, nullptr);

		contents = g_csgo.m_engine_trace->GetPointContents(out, CS_MASK_SHOOT | CONTENTS_HITBOX, nullptr);

		if ((contents & CS_MASK_SHOOT) && (!(contents & CONTENTS_HITBOX) || (contents == first_contents)))
			continue;

		// move end pos a bit for tracing.
		new_end = out - (dir * 4.f);

		// do first trace.
		g_csgo.m_engine_trace->TraceRay(Ray(out, new_end), CS_MASK_SHOOT | CONTENTS_HITBOX, nullptr, exit_trace);

		// we hit an ent's hitbox, do another trace.
		if (exit_trace->m_startsolid && (exit_trace->m_surface.m_flags & SURF_HITBOX)) {
			filter.SetPassEntity(exit_trace->m_entity);

			g_csgo.m_engine_trace->TraceRay(Ray(out, start), CS_MASK_SHOOT, (ITraceFilter*)&filter, exit_trace);

			if (exit_trace->hit() && !exit_trace->m_startsolid) {
				out = exit_trace->m_endpos;
				return true;
			}

			continue;
		}

		else if (!exit_trace->hit() || exit_trace->m_startsolid) {
			if (game::IsBreakable(enter_trace->m_entity)) {
				*exit_trace = *enter_trace;
				exit_trace->m_endpos = start + dir;
				return true;
			}

			continue;
		}

		else if ((exit_trace->m_surface.m_flags & SURF_NODRAW)) {
			// note - dex; ok, when this happens the game seems to not ignore world?
			if (game::IsBreakable(exit_trace->m_entity) && game::IsBreakable(enter_trace->m_entity)) {
				out = exit_trace->m_endpos;
				return true;
			}

			if (!(enter_trace->m_surface.m_flags & SURF_NODRAW))
				continue;
		}

		else if (exit_trace->m_plane.m_normal.dot(dir) <= 1.f) {
			out -= (dir * (exit_trace->m_fraction * 4.f));
			return true;
		}

	} while (dist <= 90.f);

	return false;
}


void penetration::ClipTraceToPlayer(const vec3_t vecAbsStart, const vec3_t& vecAbsEnd, uint32_t iMask, ITraceFilter* pFilter, CGameTrace* pGameTrace, Player* player) {
	vec3_t     pos, to, dir, on_ray;
	float      len, range_along, range;
	Ray        ray;
	CGameTrace new_trace;

	// reference: https://github.com/alliedmodders/hl2sdk/blob/3957adff10fe20d38a62fa8c018340bf2618742b/game/shared/util_shared.h#L381

	// set some local vars.
	pos = player->m_vecOrigin() + ((player->m_vecMins() + player->m_vecMaxs()) * 0.5f);
	to = pos - vecAbsStart;
	dir = vecAbsStart - vecAbsEnd;
	len = dir.normalize();
	range_along = dir.dot(to);

	// off start point.
	if (range_along < 0.f)
		range = -(to).length();

	// off end point.
	else if (range_along > len)
		range = -(pos - vecAbsEnd).length();

	// within ray bounds.
	else {
		on_ray = vecAbsStart + (dir * range_along);
		range = (pos - on_ray).length();
	}

	if ( /*min <= range &&*/ range <= 60.f) {
		// clip to player.
		g_csgo.m_engine_trace->ClipRayToEntity(Ray(vecAbsStart, vecAbsEnd), iMask, player, &new_trace);

		if (pGameTrace->m_fraction > new_trace.m_fraction)
			*pGameTrace = new_trace;
	}
}

bool penetration::run(PenetrationInput_t* in, PenetrationOutput_t* out) {

	CTraceFilterSimple_game filter{};
	int			  pen{ in->m_center ? 4 : 3 }, enter_material, exit_material;
	float		  damage, penetration, penetration_mod, player_damage, remaining, trace_len{}, total_pen_mod, damage_mod, modifier, damage_lost;
	surfacedata_t* enter_surface, * exit_surface;
	bool		  nodraw, grate;
	vec3_t		  start, dir, end, pen_end;
	CGameTrace	  trace, exit_trace;
	Weapon* weapon;
	WeaponInfo* weapon_info;

	// if we are tracing from our local player perspective.
	if (in->m_from == g_cl.m_local) {
		weapon = g_cl.m_weapon;
		weapon_info = g_cl.m_weapon_info;
		start = g_cl.m_shoot_pos;
	}

	// not local player.
	else {
		weapon = in->m_from->GetActiveWeapon();
		if (!weapon)
			return false;

		// get weapon info.
		weapon_info = weapon->GetWpnData();
		if (!weapon_info)
			return false;

		// set trace start.
		start = in->m_from->GetShootPosition();
	}

	if (!weapon)
		return false;

	if (!weapon_info)
		return false;

	// reset this
	out->m_end_in_solid = false;
	out->m_last_pen_count = 4;
	out->m_damage = -1.f;

	// get some weapon data.
	damage = (float)weapon_info->m_damage;
	penetration = weapon_info->m_penetration;

	// used later in calculations.
	penetration_mod = std::max(0.f, (3.f / penetration) * 1.25f);

	// get direction to end point.
	dir = (in->m_pos - start).normalized();

	// setup trace filter for later.
	filter.SetPassEntity(in->m_from);

	while (damage > 0.f) {
		// calculating remaining len.
		remaining = weapon_info->m_range - trace_len;

		// set trace end.
		end = start + (dir * remaining);

		// update dis
		out->m_last_pen_count = pen;

		// setup ray and trace.
		// TODO; use UTIL_TraceLineIgnoreTwoEntities?
		g_csgo.m_engine_trace->TraceRay(Ray(start, end), CS_MASK_SHOOT | CONTENTS_HITBOX, (ITraceFilter*)&filter, &trace);

		// we didn't hit anything.
		if (trace.m_fraction == 1.f) {
			//	out->m_end_in_solid = true;
			return false;
		}


		// check for player hitboxes extending outside their collision bounds.
		// if no target is passed we clip the trace to a specific player, otherwise we clip the trace to any player.
		if (in->m_target)
			// ClipTraceToPlayer(start, end + (dir * 40.f), CS_MASK_SHOOT | CONTENTS_HITBOX, &trace, in->m_target, -60.f);
			ClipTraceToPlayer(start, end + (dir * 40.f), CS_MASK_SHOOT | CONTENTS_HITBOX, (ITraceFilter*)&filter, &trace, in->m_target);
		else
			game::UTIL_ClipTraceToPlayers(start, end + (dir * 40.f), CS_MASK_SHOOT | CONTENTS_HITBOX, (ITraceFilter*)&filter, &trace, -60.f);





		// calculate damage based on the distance the bullet traveled.
		trace_len += trace.m_fraction * remaining;
		damage *= std::pow(weapon_info->m_range_modifier, trace_len / 500.f);

		// if a target was passed.
		if (in->m_target) {

			// validate that we hit the target we aimed for.
			if (trace.m_entity && trace.m_entity == in->m_target && game::IsValidHitgroup(trace.m_hitgroup)) {
				int group = (weapon->m_iItemDefinitionIndex() == ZEUS) ? HITGROUP_GENERIC : trace.m_hitgroup;

				// scale damage based on the hitgroup we hit.
				player_damage = scale(in->m_target, damage, weapon_info->m_armor_ratio, group);

				// set result data for when we hit a player.
				out->m_pen = pen != 4;
				out->m_hitgroup = group;
				out->m_damage = player_damage;
				out->m_target = in->m_target;

				// non-penetrate damage.
				if (pen == 4)
					return (player_damage >= in->m_damage || player_damage >= in->m_target->m_iHealth());

				// penetration damage.
				return (player_damage >= in->m_damage_pen || player_damage >= in->m_target->m_iHealth());
			}
		}

		// no target was passed, check for any player hit or just get final damage done.
		else {
			out->m_pen = pen != 4;

			// todo - dex; team checks / other checks / etc.
			if (trace.m_entity && trace.m_entity->IsPlayer() && game::IsValidHitgroup(trace.m_hitgroup)) {
				int group = (weapon->m_iItemDefinitionIndex() == ZEUS) ? HITGROUP_GENERIC : trace.m_hitgroup;

				player_damage = scale(trace.m_entity->as< Player* >(), damage, weapon_info->m_armor_ratio, group);

				// set result data for when we hit a player.
				out->m_hitgroup = group;
				out->m_damage = player_damage;
				out->m_target = trace.m_entity->as< Player* >();

				// non-penetrate damage.
				if (pen == 4)
					return (player_damage >= in->m_damage || player_damage >= out->m_target->m_iHealth());

				// penetration damage.
				return (player_damage >= in->m_damage_pen || player_damage >= out->m_target->m_iHealth());
			}

			// if we've reached here then we didn't hit a player yet, set damage and hitgroup.
			out->m_damage = damage;
		}

		// don't run pen code if it's not wanted.
		if (!in->m_can_pen)
			return false;

		// get surface at entry point.
		enter_surface = g_csgo.m_phys_props->GetSurfaceData(trace.m_surface.m_surface_props);

		// this happens when we're too far away from a surface and can penetrate walls or the surface's pen modifier is too low.
		if ((trace_len > 3000.f && penetration) || enter_surface->m_game.m_penetration_modifier < 0.1f)
			return false;

		// store data about surface flags / contents.
		nodraw = (trace.m_surface.m_flags & SURF_NODRAW);
		grate = (trace.m_contents & CONTENTS_GRATE);

		// get material at entry point.
		enter_material = enter_surface->m_game.m_material;

		// note - dex; some extra stuff the game does.
		if (!pen && !nodraw && !grate && enter_material != CHAR_TEX_GRATE && enter_material != CHAR_TEX_GLASS)
			return false;

		// no more pen.
		if (penetration <= 0.f || pen <= 0)
			return false;



		// try to penetrate object.
		if (!TraceToExit(&trace, trace.m_endpos, dir, &exit_trace)) {
			if (!(g_csgo.m_engine_trace->GetPointContents(pen_end, CS_MASK_SHOOT) & CS_MASK_SHOOT)) {
				out->m_end_in_solid = true;
				return false;
			}
		}


		// get surface / material at exit point.
		exit_surface = g_csgo.m_phys_props->GetSurfaceData(exit_trace.m_surface.m_surface_props);
		exit_material = exit_surface->m_game.m_material;

		// todo - dex; check for CHAR_TEX_FLESH and ff_damage_bullet_penetration / ff_damage_reduction_bullets convars?
		//             also need to check !isbasecombatweapon too.
		if (enter_material == CHAR_TEX_GRATE || enter_material == CHAR_TEX_GLASS) {
			total_pen_mod = 3.f;
			damage_mod = 0.05f;
		}

		else if (nodraw || grate) {
			total_pen_mod = 1.f;
			damage_mod = 0.16f;
		}

		else {
			total_pen_mod = (enter_surface->m_game.m_penetration_modifier + exit_surface->m_game.m_penetration_modifier) * 0.5f;
			damage_mod = 0.16f;
		}

		// thin metals, wood and plastic get a penetration bonus.
		if (enter_material == exit_material) {
			if (exit_material == CHAR_TEX_CARDBOARD || exit_material == CHAR_TEX_WOOD)
				total_pen_mod = 3.f;

			else if (exit_material == CHAR_TEX_PLASTIC)
				total_pen_mod = 2.f;
		}

		// set some local vars.
		// NOTE: this is a test but game uses like float flDistance, not float&
		// meaning it doesnt actually edit the float lol
		float trace_len2 = (exit_trace.m_endpos - trace.m_endpos).length();
		modifier = std::max(1.f / total_pen_mod, 0.f);

		// this calculates how much damage we've lost depending on thickness of the wall, our penetration, damage, and the modifiers set earlier
		float percent_damage_chunk = damage * damage_mod;
		float pen_wep_mod = percent_damage_chunk + std::max(0.f, (3.f / penetration) * 1.25f) * (modifier * 3.f);
		float lost_damage_obj = ((modifier * (trace_len2 * trace_len2)) / 24.f);
		float total_lost_dam = pen_wep_mod + lost_damage_obj;

		// subtract from damage.
		damage -= std::max(0.f, total_lost_dam);
		if (damage < 1.f) {
			out->m_end_in_solid = true;
			return false;
		}

		// set new start pos for successive trace.
		start = exit_trace.m_endpos;

		// decrement pen.
		--pen;
	}

	return false;
}



bool penetration::TraceToExit(CGameTrace* pEnterTrace, vec3_t vecStartPos, vec3_t vecDirection, CGameTrace* pExitTrace) {
	constexpr float flMaxDistance = 90.f, flStepSize = 4.f;
	float flCurrentDistance = 0.f;

	int iFirstContents = 0;

	bool bIsWindow = 0;
	auto v23 = 0;

	do {
		// Add extra distance to our ray
		flCurrentDistance += flStepSize;

		// Multiply the direction vector to the distance so we go outwards, add our position to it.
		vec3_t vecEnd = vecStartPos + (vecDirection * flCurrentDistance);

		if (!iFirstContents)
			iFirstContents = g_csgo.m_engine_trace->GetPointContents(vecEnd, CS_MASK_SHOOT | CONTENTS_HITBOX);

		int iPointContents = g_csgo.m_engine_trace->GetPointContents(vecEnd, CS_MASK_SHOOT | CONTENTS_HITBOX);

		if (!(iPointContents & CS_MASK_SHOOT) || ((iPointContents & CONTENTS_HITBOX) && iPointContents != iFirstContents)) {

			//Let's setup our end position by deducting the direction by the extra added distance
			vec3_t vecStart = vecEnd - (vecDirection * flStepSize);

			// this gets a bit more complicated and expensive when we have to deal with displacements
			TraceLine(vecEnd, vecStart, CS_MASK_SHOOT | CONTENTS_HITBOX, nullptr, pExitTrace);

			// note - dex; this is some new stuff added sometime around late 2017 ( 10.31.2017 update? ).
			//if ( g_Vars.sv_clip_penetration_traces_to_players->GetInt( ) )
			//	ClipTraceToPlayers( vecEnd, vecStart, MASK_SHOT_PLAYER, nullptr, pExitTrace, -60.f, -1.f );

			// we hit an ent's hitbox, do another trace.
			if (pExitTrace->m_startsolid && pExitTrace->m_surface.m_flags & SURF_HITBOX) {


				CTraceFilterSimple_game filter_;
				filter_.SetPassEntity(pExitTrace->m_entity);


				// do another trace, but skip the player to get the actual exit surface 
				TraceLine(vecStartPos, vecStart, MASK_SHOT_HULL, (ITraceFilter*)&filter_, pExitTrace);

				if (pExitTrace->hit() && !pExitTrace->m_startsolid) {
					vecEnd = pExitTrace->m_endpos;
					return true;
				}

				continue;
			}

			//Can we hit? Is the wall solid?
			else if (pExitTrace->hit() && !pExitTrace->m_startsolid) {
				if (game::IsBreakable(pEnterTrace->m_entity) && game::IsBreakable(pExitTrace->m_entity))
					return true;

				if (pEnterTrace->m_surface.m_flags & SURF_NODRAW ||
					(!(pExitTrace->m_surface.m_flags & SURF_NODRAW) && pExitTrace->m_plane.m_normal.Dot(vecDirection) <= 1.f)) {
					const float flMultAmount = pExitTrace->m_fraction * 4.f;

					// get the real end pos
					vecStart -= vecDirection * flMultAmount;
					return true;
				}

				continue;
			}

			if (!pExitTrace->hit() || pExitTrace->m_startsolid) {
				if (pEnterTrace->DidHitNonWorldEntity() && game::IsBreakable(pEnterTrace->m_entity)) {
					// if we hit a breakable, make the assumption that we broke it if we can't find an exit (hopefully..)
					// fake the end pos
					pExitTrace = pEnterTrace;
					pExitTrace->m_endpos = vecStartPos + vecDirection;
					return true;
				}
			}
		}
		// max pen distance is 90 units.
	} while (flCurrentDistance <= flMaxDistance);

	return false;
}