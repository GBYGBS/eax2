#pragma once

namespace penetration {
	struct PenetrationInput_t {
		Player* m_from;
		Player* m_target;
		vec3_t  m_pos;
		float	m_damage;
		float   m_damage_pen;
		bool	m_can_pen;
		bool    m_center;
	};

	struct PenetrationOutput_t {
		Player* m_target;
		float   m_damage;
		int     m_hitgroup;
		bool    m_pen;
		bool    m_end_in_solid;
		int     m_last_pen_count;
		__forceinline PenetrationOutput_t() : m_target{ nullptr }, m_damage{ 0.f }, m_hitgroup{ -1 }, m_pen{ false }, m_end_in_solid{ false } {}
	};

    float scale( Player* player, float damage, float armor_ratio, int hitgroup );
    bool  TraceToExit( const vec3_t& start, const vec3_t& dir, vec3_t& out, CGameTrace* enter_trace, CGameTrace* exit_trace );
    bool  run( PenetrationInput_t* in, PenetrationOutput_t* out );
	void ClipTraceToPlayer(const vec3_t vecAbsStart, const vec3_t& vecAbsEnd, uint32_t iMask, ITraceFilter* pFilter, CGameTrace* pGameTrace, Player* player);
	bool TraceToExit(CGameTrace* pEnterTrace, vec3_t vecStartPos, vec3_t vecDirection, CGameTrace* pExitTrace);
}