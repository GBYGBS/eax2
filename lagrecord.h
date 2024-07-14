#pragma once
#include <optional>

// pre-declare.
class LagRecord;

class BackupRecord {
public:
	BoneArray m_bones[128];
	int        m_bone_count;
	vec3_t     m_origin, m_abs_origin;
	vec3_t     m_mins;
	vec3_t     m_maxs;
	ang_t      m_abs_ang;

public:
	__forceinline void store(Player* player) {

		m_bone_count = player->m_BoneCache().m_CachedBoneCount;
		memcpy(m_bones, player->m_BoneCache().m_pCachedBones, m_bone_count * sizeof(matrix3x4_t));

		m_origin = player->m_vecOrigin();
		m_mins = player->m_vecMins();
		m_maxs = player->m_vecMaxs();
		m_abs_origin = player->GetAbsOrigin();
		m_abs_ang = player->GetAbsAngles();
	}

	__forceinline void restore(Player* player) {


		memcpy(player->m_BoneCache().m_pCachedBones, m_bones, m_bone_count * sizeof(matrix3x4_t));


		player->m_vecOrigin() = m_origin;
		player->m_vecMins() = m_mins;
		player->m_vecMaxs() = m_maxs;
		player->SetAbsAngles(m_abs_ang);
		player->SetAbsOrigin(m_origin);
	}
};

enum shift_type_t {
	SHIFT_NONE = 0,
	SHIFT_DEFAULT,
	SHIFT_BREAK_LC
};

class LagRecord {
public:
	// data.
	Player* m_player;
	float   m_immune;
	int     m_tick;
	int     m_lag, m_anim_lag;
	float   m_lag_time;
	float   m_realtime;
	bool    m_dormant;
	int m_shift;

	// netvars.
	float  m_sim_time;
	float  m_old_sim_time;
	int    m_flags;
	vec3_t m_origin;
	vec3_t m_old_origin;
	vec3_t m_velocity;
	vec3_t m_mins;
	vec3_t m_maxs;
	float m_new_body;
	ang_t  m_eye_angles;
	ang_t  m_abs_ang;
	float  m_body;
	float  m_duck;
	std::string m_resolver_mode;
	std::string m_resolver_mode2;
	Color m_resolver_color;
	bool  resolved;

	// anim stuff.
	C_AnimationLayer		m_layers[13];
	float					m_poses[24];
	vec3_t					m_anim_velocity, m_base_vel;
	float					m_on_ground_time{};
	int					    m_ground_state{};


	// bone stuff.
	bool       m_setup;
	BoneArray m_bones[128];
	BoneArray m_extrap_bones[128];

	// lagfix stuff.
	bool   m_broke_lc;
	vec3_t m_pred_origin;
	vec3_t m_pred_velocity;
	float  m_pred_time;
	int    m_pred_flags;
	float m_feet_yaw;
	std::optional < bool > m_unfixed_on_ground{};
	bool   m_ground;
	float m_valid_time;

	// resolver stuff.
	size_t m_mode;
	bool   m_shot;
	bool   m_fake_shot;
	float  m_away;
	float  m_anim_time;
	C_AnimationLayer m_server_layers[13];
	float  m_animation_speed, m_max_speed;
	bool   m_fake_walk, m_fake_flick;

	float	m_foot_yaw{}, m_move_yaw{};
	float m_move_yaw_cur_to_ideal{};
	float m_move_yaw_ideal{};
	float m_move_weight_smoothed{};
	int              m_server_delta;

	// other stuff.
	float  m_interp_time;
	bool   m_sideway;
	bool   m_invalid;
	float  m_choke_time;
	Weapon* m_weapon;
	float m_shot_time;
	float m_act_time{ };
	bool  m_extrapolated;
	bool  m_ground_for_two_ticks;
	bool		m_has_seen_delta_35;

	float       m_back;

	int         m_shot_type;

public:

	__forceinline int shift_type() {

		int ret = shift_type_t::SHIFT_NONE;
		// most people dont shift with fakelag so it's useless
		if (m_lag > 3)
			return ret;

		// if delta too high or delta low but he's teleporting
		if (m_server_delta <= -10 || (m_origin - m_old_origin).length_sqr() > 4096.f)
			ret = shift_type_t::SHIFT_BREAK_LC;
		// delta but not breaking lc ( normal shift )
		else if (m_server_delta < -1)
			ret = shift_type_t::SHIFT_DEFAULT;

		return ret;
	}


	// default ctor.
	__forceinline LagRecord() :
		m_setup{ false },
		m_broke_lc{ false },
		m_fake_walk{ false },
		m_sideway{ false },
		m_shot{ false },
		m_has_seen_delta_35{ false },
		m_back{ 0 },
		m_server_delta{ 0 },
		m_shot_type{ 0 },
		m_lag{},
		m_lag_time{} {}

	// ctor.
	__forceinline LagRecord(Player* player) :
		m_setup{ false },
		m_broke_lc{ false },
		m_fake_walk{ false },
		m_sideway{ false },
		m_shot{ false },
		m_has_seen_delta_35{ false },
		m_back{ 0 },
		m_server_delta{ 0 },
		m_shot_type{ 0 },
		m_lag{},
		m_lag_time{} {

		store(player);
	}


	__forceinline void invalidate() {

		// mark as not setup.
		m_setup = false;

	}

	// function: allocates memory for SetupBones and stores relevant data.
	void store(Player* player) {
		// allocate game heap.

		// player data.
		m_realtime = g_csgo.m_globals->m_realtime;
		m_player = player;
		m_immune = player->m_fImmuneToGunGameDamageTime();
		m_tick = g_csgo.m_cl->m_server_tick;

		// netvars.
		m_pred_time = m_sim_time = player->m_flSimulationTime();
		m_old_sim_time = player->m_flOldSimulationTime();
		m_pred_flags = m_flags = player->m_fFlags();
		m_pred_origin = m_origin = player->m_vecOrigin();
		m_old_origin = player->m_vecOldOrigin();
		m_eye_angles = player->m_eye_angles();
		m_abs_ang = player->GetAbsAngles();
		m_body = player->m_body();
		m_mins = player->m_vecMins();
		m_maxs = player->m_vecMaxs();
		m_duck = player->m_flDuckAmount();
		m_base_vel = m_pred_velocity = m_velocity = m_anim_velocity = player->m_vecVelocity();
		m_weapon = player->GetActiveWeapon();
		m_shot_time = m_weapon ? m_weapon->m_fLastShotTime() : 0.f;
		m_max_speed = 250.f;

		// save networked animlayers.
		player->GetAnimLayers(m_layers);

		// normalize eye angles.
		m_eye_angles.normalize();
		math::clamp(m_eye_angles.x, -90.f, 90.f);

		// get lag.
		m_lag_time = m_sim_time - m_old_sim_time; // lagcomp time, its just so we got smth incase it goes wrong
		m_lag = game::TIME_TO_TICKS(m_sim_time - m_old_sim_time);

		m_server_delta = game::TIME_TO_TICKS(m_sim_time) - g_csgo.m_cl->m_server_tick;


		// compute animtime.
		m_anim_time = m_old_sim_time + g_csgo.m_globals->m_interval;

		if (const auto anim_state = player->m_PlayerAnimState())
			m_foot_yaw = anim_state->m_foot_yaw;
	}

	// function: restores 'predicted' variables to their original.
	__forceinline void predict() {
		m_broke_lc = broke_lc();
		m_pred_origin = m_origin;
		m_pred_velocity = m_velocity;
		m_pred_time = m_sim_time;
		m_pred_flags = m_flags;
		m_extrapolated = false;

		// memcpy bones
		if (m_setup)
			memcpy(m_extrap_bones, m_bones, m_player->m_BoneCache().m_CachedBoneCount * sizeof(matrix3x4_t));
	}

	// function: writes current record to bone cache.
	__forceinline void cache() {


		if (m_setup && !m_dormant) {
			m_player->InvalidateBoneCache();

			if (!m_extrapolated)
				memcpy(m_player->m_BoneCache().m_pCachedBones, m_bones, m_player->m_BoneCache().m_CachedBoneCount * sizeof(matrix3x4_t));
			else
				memcpy(m_player->m_BoneCache().m_pCachedBones, m_extrap_bones, m_player->m_BoneCache().m_CachedBoneCount * sizeof(matrix3x4_t));
		}

		m_player->m_vecOrigin() = m_pred_origin;
		m_player->m_vecMins() = m_mins;
		m_player->m_vecMaxs() = m_maxs;

		m_player->SetAbsAngles(m_abs_ang);
		m_player->SetAbsOrigin(m_pred_origin);
	}

	__forceinline bool dormant() {
		return m_dormant;
	}

	__forceinline bool immune() {
		return m_immune > 0.f;
	}

	__forceinline bool broke_lc() {
		return (this->m_origin - this->m_old_origin).length_2d_sqr() > 4096.f;
	}

	__forceinline void validate_lag() {

		if (m_lag > 19)
			m_lag = 1;


		if (m_anim_lag > 19)
			m_anim_lag = 1;

	}

	__forceinline float get_time_delta() {

		INetChannel* info = g_csgo.m_engine->GetNetChannelInfo();
		if (!info || !g_cl.m_local)
			return -1.f;

		// get correct based on out latency + in latency + lerp time and clamp on sv_maxunlag
		float correct = 0.f;

		// add out latency
		correct += info->GetLatency(INetChannel::FLOW_OUTGOING);

		// add in latency
		correct += info->GetLatency(INetChannel::FLOW_INCOMING);

		// add interpolation amount
		correct += g_cl.m_lerp;

		// clamp this shit
		correct = std::clamp(correct, 0.f, g_csgo.sv_maxunlag->GetFloat());

		// def cur time
		float curtime = g_cl.m_local->alive() ? game::TICKS_TO_TIME(g_cl.m_local->m_nTickBase()) : g_csgo.m_globals->m_curtime;

		// simtime -> tickcount -> target time
		float target_time = this->m_sim_time;

		// get delta time
		// lol this looks cancer but thats the way it is ._.
		// https://github.com/perilouswithadollarsign/cstrike15_src/blob/f82112a2388b841d72cb62ca48ab1846dfcc11c8/game/server/player_lagcompensation.cpp#L287
		return std::fabs(correct - (curtime - target_time));
	}

	// function: checks if LagRecord obj is hittable if we were to fire at it now.
	__forceinline bool valid() {

		INetChannel* info = g_csgo.m_engine->GetNetChannelInfo();
		if (!info || !g_cl.m_local)
			return false;

		// defensive fix / unsetupped fix
		if (!m_setup)
			return false;

		// if out of range
		// https://github.com/perilouswithadollarsign/cstrike15_src/blob/f82112a2388b841d72cb62ca48ab1846dfcc11c8/game/server/player_lagcompensation.cpp#L292
		if (get_time_delta() > 0.2f) // 0.19f -> smaller backtrack but fixes rounding issues
			return false;

		// just ignore dead time or not?
		return true;
	}
};