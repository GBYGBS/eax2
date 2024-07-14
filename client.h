#pragma once

class Sequence {
public:
	float m_time;
	int   m_state;
	int   m_seq;

public:
	__forceinline Sequence() : m_time{}, m_state{}, m_seq{} {};
	__forceinline Sequence(float time, int state, int seq) : m_time{ time }, m_state{ state }, m_seq{ seq } {};
};

class NetPos {
public:
	float  m_time;
	vec3_t m_pos;

public:
	__forceinline NetPos() : m_time{}, m_pos{} {};
	__forceinline NetPos(float time, vec3_t pos) : m_time{ time }, m_pos{ pos } {};
};

class Client {
public:
	// hack thread.
	static ulong_t __stdcall init(void* arg);

	void UnlockHiddenConvars();

	void StartMove(CUserCmd* cmd);
	void EndMove(CUserCmd* cmd);
	void BackupPlayers(bool restore);
	void DoMove();
	void DrawHUD();
	void UpdateInformation();
	void MouseFix(CUserCmd* cmd);
	void SetAngles();
	void OnCreateMove();
	void KillFeed();
	void MotionBlur();

	void OnPaint();
	void OnMapload();
	void OnTick(CUserCmd* cmd);

	// debugprint function.
	void print(const std::string text, ...);

	// check if we are able to fire this tick.
	bool CanFireWeapon();
	void UpdateRevolverCock();
	void UpdateIncomingSequences();

public:
	// local player variables.
	Player* m_local;
	bool	         m_processing;
	bool	         m_setupped, m_updated_values;
	int	             m_flags;
	bool             m_hit_floor;
	vec3_t	         m_shoot_pos;
	bool	         m_player_fire;
	bool	         m_shot;
	bool	         m_old_shot;
	float            m_abs_yaw;
	float            m_poses[24];
	C_AnimationLayer m_layers[13];
	C_AnimationLayer m_backup_layers[13];
	vec3_t           m_last_sent_origin;
	float            m_upd_time;
	int              m_upd_tick;
	float            m_spawn_time;
	bool             m_has_updated;
	int              m_upd_time_test;
	float            m_last_sim_time;
	bool             m_real_update;

	ang_t            m_real_angle;
	bool m_update_anims;
	bool             m_pressing_move;

	// active weapon variables.
	Weapon* m_weapon;
	int         m_weapon_id;
	WeaponInfo* m_weapon_info;
	int         m_weapon_type;
	vec3_t m_unpredicted_vel;
	bool        m_weapon_fire;

	BoneArray        m_local_bones[128];
	BoneArray        m_matrix[128];

	// revolver variables.
	int	 m_revolver_cock;
	int	 m_revolver_query;
	bool m_revolver_fire;

	// general game varaibles.
	bool     m_round_end;
	Stage_t	 m_stage;
	int	     m_max_lag;
	int      m_lag;
	int	     m_old_lag;
	bool     m_should_try_upd;
	bool* m_packet;
	bool* m_final_packet;
	bool	 m_old_packet;
	float	 m_lerp;
	float    m_latency;
	float    m_latency2;
	int      m_latency_ticks;
	int      m_server_tick;
	int      m_arrival_tick;
	int      m_width, m_height;
	bool m_sideways;
	// usercommand variables.
	CUserCmd* m_cmd;
	int	      m_tick;
	int	      m_buttons;
	int       m_old_buttons;
	ang_t     m_view_angles;
	ang_t	  m_strafe_angles;
	bool m_allow_client_to_update;
	vec3_t	  m_forward_dir;

	penetration::PenetrationOutput_t m_pen_data;

	std::deque< Sequence > m_sequences;
	std::deque< NetPos >   m_net_pos;

	// animation variables.
	ang_t  m_angle;
	ang_t  m_rotation;
	ang_t  m_radar;
	float  m_body;
	float  m_body_pred;
	float  m_speed;
	float  m_anim_time;
	float  m_anim_frame;
	int    m_frame_shit;
	bool   m_ground;
	bool   m_lagcomp;

	std::vector<int> cheese;
	std::vector<int> kaaba;
	std::vector<int> dopium;
	std::vector<int> roberthook;
	std::vector<int> fade;
	std::vector<int> test;
	std::vector<int> same_hack;

	bool m_enable_voice;

	// aici o fut pe masa lu wake. 
	float m_flPreviousDuckAmount = 0.0f;

	// hack info.
	std::string m_build = "dev";
	std::string m_userr = "dev";
	bool m_hwid_found = false;
	int id = 133769;

	float backup_poses[24];
	C_AnimationLayer backup_layers[13];
	matrix3x4_t backup_cache[128];
	matrix3x4_t backup_acc[128];
	int backup_count;
};

extern Client g_cl;