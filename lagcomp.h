#pragma once

class AimPlayer;

struct extrapolation_data_t {
	__forceinline constexpr extrapolation_data_t() = default;

	__forceinline extrapolation_data_t(
		Player* const player, const LagRecord* const lag_record
	) : m_player{ player }, m_sim_time{ lag_record->m_sim_time }, m_flags{ lag_record->m_flags },
		m_was_in_air{ !(lag_record->m_flags & FL_ONGROUND) }, m_origin{ lag_record->m_origin },
		m_velocity{ lag_record->m_velocity }, m_obb_min{ lag_record->m_mins }, m_obb_max{ lag_record->m_maxs } {}

	Player* m_player{};

	float				m_sim_time{};

	int					m_flags{};
	bool				m_was_in_air{};

	vec3_t				m_origin{}, m_velocity{},
		m_obb_min{}, m_obb_max{};
};

class Extrapolation {
public:
	bool HandleLagCompensation(AimPlayer* player);
	void SimulateMovement(extrapolation_data_t& data);
};

extern Extrapolation g_extrapolation;