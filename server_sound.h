#pragma once

class ServerSound {
public:
	// Call before and after ESP.
	void Start();

	bool AdjustPlayerBegin(Player* player);
	void SetupAdjustPlayer(Player* player, IEngineSound::SoundInfo& sound);

	bool ValidSound(IEngineSound::SoundInfo& sound);

	struct SoundPlayer {
		void reset(bool store_data = false, const vec3_t& origin = vec3_t(0, 0, 0), int flags = 0) {
			if (store_data) {
				m_iReceiveTime = g_csgo.m_globals->m_realtime;
				m_vecOrigin = origin;
				m_nFlags = flags;
			}
			else {
				m_iReceiveTime = 0.0f;
				m_vecOrigin.clear();
				m_nFlags = 0;
			}
		}

		void Override(IEngineSound::SoundInfo& sound) {
			m_iIndex = sound.sound_source;
			m_vecOrigin = *sound.origin;
			m_iReceiveTime = g_csgo.m_globals->m_realtime;
		}

		int m_iIndex = 0;
		int m_iReceiveTime = 0;
		vec3_t m_vecOrigin = vec3_t(0, 0, 0);
		/* Restore data */
		int m_nFlags = 0;
		Player* player = nullptr;
		vec3_t m_vecAbsOrigin = vec3_t(0, 0, 0);
		bool m_bDormant = false;
	} m_cSoundPlayers[64];
	CUtlVector<IEngineSound::SoundInfo> m_utlvecSoundBuffer;
	CUtlVector<IEngineSound::SoundInfo> m_utlCurSoundList;
	std::vector<SoundPlayer> m_arRestorePlayers;
};

extern ServerSound g_sound;