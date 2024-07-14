#pragma once

class ShotRecord {
public:
	__forceinline ShotRecord() : m_target{}, m_record{}, m_time{}, m_lat{}, m_damage{}, m_aim_point{}, m_range{}, m_pos{}, m_impact_pos{}, m_confirmed{}, m_had_pred_error{}, m_hurt{}, m_impacted{}, m_hitbox{}, m_hitgroup{} {}

public:
	Player* m_target;
	LagRecord* m_record;
	float      m_time, m_lat, m_damage, m_range;
	vec3_t     m_pos, m_impact_pos;
	bool       m_confirmed, m_hurt, m_impacted, m_had_pred_error;
	vec3_t      m_aim_point;
	int		   m_hitbox;
	int        m_hitgroup;
	bool       m_invalid_record;
};

class VisualImpactData_t {
public:
	vec3_t m_impact_pos, m_shoot_pos;
	int    m_tickbase;
	bool   m_ignore, m_hit_player;

public:
	__forceinline VisualImpactData_t(const vec3_t& impact_pos, const vec3_t& shoot_pos, int tickbase) :
		m_impact_pos{ impact_pos }, m_shoot_pos{ shoot_pos }, m_tickbase{ tickbase }, m_ignore{ false }, m_hit_player{ false } {}
};

class ImpactRecord {
public:
	__forceinline ImpactRecord() : m_shot{}, m_pos{}, m_tick{} {}

public:
	ShotRecord* m_shot;
	int         m_tick;
	vec3_t      m_pos;
};

class HitRecord {
public:
	__forceinline HitRecord() : m_impact{}, m_group{ -1 }, m_damage{} {}

public:
	ImpactRecord* m_impact;
	int           m_group;
	float         m_damage;
};

class Shots {
public:
	std::array< std::string, 8 > m_groups = {
	   XOR("body"),
	   XOR("head"),
	   XOR("chest"),
	   XOR("stomach"),
	   XOR("left arm"),
	   XOR("right arm"),
	   XOR("left leg"),
	   XOR("right leg")
	};

	void OnShotFire(Player* target, float damage, int bullets, LagRecord* record, vec3_t aim_point, int hitbox, int hitgroup);
	void OnImpact(IGameEvent* evt);
	void OnHurt(IGameEvent* evt);
	void OnWeaponFire(IGameEvent* evt);
	void OnShotMiss(ShotRecord& shot);
	void Think();

public:
	std::deque< ShotRecord >          m_shots;
	std::vector< VisualImpactData_t > m_vis_impacts;

	float iHitDmg = NULL;
	bool iHit = false;
	int	taps;
	bool iHeadshot = false;
	bool canhit = false;

	vec3_t iPlayerOrigin, iPlayermins, iPlayermaxs;
	vec2_t iPlayerbottom, iPlayertop;
};

extern Shots g_shots;