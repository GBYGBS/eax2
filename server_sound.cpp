#include "includes.h"

ServerSound g_sound{ };;

bool valid(Player* player, bool check_team, bool check_dormant) {
	if (!player)
		return false;

	if (!g_cl.m_local)
		return false;

	if (!player->IsPlayer())
		return false;

	if (!player->alive())
		return false;

	if (player->dormant() && check_dormant)
		return false;

	if (check_team && g_cl.m_local->m_iTeamNum() == player->m_iTeamNum())
		return false;

	return true;
}

void ServerSound::Start() {
	m_utlCurSoundList.RemoveAll();
	g_csgo.m_sound->GetActiveSounds(m_utlCurSoundList);

	if (!m_utlCurSoundList.Count())
		return;

	for (int iter = 0; iter < m_utlCurSoundList.Count(); iter++) {
		IEngineSound::SoundInfo& sound = m_utlCurSoundList[iter];
		if (sound.sound_source < 1 || sound.sound_source > 64)
			continue;

		if (sound.origin->is_zero())
			continue;

		if (!ValidSound(sound))
			continue;

		Player* player = (Player*)g_csgo.m_entlist->GetClientEntity(sound.sound_source);

		if (!valid(player, true, false))
			continue;

		SetupAdjustPlayer(player, sound);

		m_cSoundPlayers[sound.sound_source].Override(sound);
	}

	m_utlvecSoundBuffer = m_utlCurSoundList;
}

void ServerSound::SetupAdjustPlayer(Player* player, IEngineSound::SoundInfo& sound) {
	vec3_t src3D, dst3D;
	CGameTrace tr;
	CTraceFilterWorldOnly filter;

	src3D = (*sound.origin) + vec3_t(0, 0, 1);
	dst3D = src3D - vec3_t(0, 0, 100);

	g_csgo.m_engine_trace->TraceRay(Ray(src3D, dst3D), MASK_PLAYERSOLID, &filter, &tr);

	if (tr.m_allsolid) {
		m_cSoundPlayers[sound.sound_source].m_iReceiveTime = -1;

		m_cSoundPlayers[sound.sound_source].m_vecOrigin = *sound.origin;
		m_cSoundPlayers[sound.sound_source].m_nFlags = player->m_fFlags();
	}
	else {
		m_cSoundPlayers[sound.sound_source].m_vecOrigin = (tr.m_fraction < 0.97 ? tr.m_endpos : *sound.origin);
		m_cSoundPlayers[sound.sound_source].m_nFlags = player->m_fFlags();
		m_cSoundPlayers[sound.sound_source].m_nFlags |= (tr.m_fraction < 0.50f ? FL_DUCKING : 0) | (tr.m_fraction != 1 ? FL_ONGROUND : 0);
		m_cSoundPlayers[sound.sound_source].m_nFlags &= (tr.m_fraction > 0.50f ? ~FL_DUCKING : 0) | (tr.m_fraction == 1 ? ~FL_ONGROUND : 0);
	}
}

bool ServerSound::AdjustPlayerBegin(Player* player) {
	auto& sound_player = m_cSoundPlayers[player->index()];

	auto expired = false;

	if (fabs(g_csgo.m_globals->m_realtime - sound_player.m_iReceiveTime) > 3.5f /* 3.5 seconds */)
		expired = true;

	player->m_fFlags() = sound_player.m_nFlags;
	player->m_vecOrigin() = sound_player.m_vecOrigin;
	player->SetAbsOrigin(sound_player.m_vecOrigin);

	return !expired;
}

bool ServerSound::ValidSound(IEngineSound::SoundInfo& sound) {

	for (auto i = 0; i < m_utlvecSoundBuffer.Count(); i++)
		if (m_utlvecSoundBuffer[i].guid == sound.guid)
			return false;

	return true;
}