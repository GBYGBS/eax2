#include "includes.h"

Extrapolation g_extrapolation{};

bool Extrapolation::HandleLagCompensation(AimPlayer* data) {
    if (!data || data->m_records.size() <= 2 || !data->m_player || data->m_player->dormant())
        return false;

    // Get the oldest record (first in vector)
    LagRecord* record = data->m_records[0].get();
    if (!record)
        return false;

    // Perform prediction reset
    record->predict();
    record->m_broke_lc = record->broke_lc();

    // If lag compensation is not broken, return false
    if (!record->m_broke_lc)
        return false;

    // Handle fakelag correction settings
    int sv_ticks_since_upd = g_cl.m_server_tick - record->m_tick;

    // Early return if we're within a certain threshold
    if (sv_ticks_since_upd <= 1 || sv_ticks_since_upd >= g_cl.m_latency_ticks || record->m_lag > 16) {
        data->m_delayed = true;
        return true;
    }

    // Calculate extrapolation based on network and server ticks
    float total_latency = g_csgo.m_net->GetLatency(0) + g_csgo.m_net->GetLatency(1);
    float time_delta = record->get_time_delta();

    int delay = g_cl.m_lag >= g_cl.m_max_lag ? 0 : 1;
    int latency_ticks = game::TIME_TO_TICKS(total_latency);
    float delta = static_cast<float>(delay + g_csgo.m_cl->m_server_tick + latency_ticks - record->m_tick) / static_cast<float>(record->m_lag);
    float max_delta = static_cast<float>(game::TIME_TO_TICKS(time_delta - 0.2f)) / static_cast<float>(record->m_lag);
    float clamped_delta = std::min(delta, max_delta);

    // Perform extrapolation for each tick
    extrapolation_data_t pred_data{ data->m_player, record };
    for (int i = 0; i < record->m_lag; ++i) {
        pred_data.m_sim_time += g_csgo.m_globals->m_interval;
        SimulateMovement(pred_data);
    }

    // Update record with extrapolated data
    record->m_extrapolated = true;
    record->m_pred_time = pred_data.m_sim_time;
    record->m_pred_flags = pred_data.m_flags;
    record->m_pred_origin = pred_data.m_origin;
    record->m_pred_velocity = pred_data.m_velocity;

    // Adjust bone positions based on extrapolated origin delta
    vec3_t origin_delta = pred_data.m_origin - record->m_origin;
    for (int i = 0; i < 128; ++i) {
        if (i < 0 || i >= 128) // Boundary check fix
            break;
        record->m_extrap_bones[i][0][3] += origin_delta.x;
        record->m_extrap_bones[i][1][3] += origin_delta.y;
        record->m_extrap_bones[i][2][3] += origin_delta.z;
    }

    return true;
}


void Extrapolation::SimulateMovement(extrapolation_data_t& data) {
    if (!data.m_player)
        return;

    // Simulate movement based on velocity and environment collisions
    if (!(data.m_flags & FL_ONGROUND)) {
        // Adjust velocity for non-ground movement (airborne)
        if (!g_csgo.sv_enablebunnyhopping || !g_csgo.sv_enablebunnyhopping->GetInt()) {
            float speed = data.m_velocity.length();
            float max_speed = data.m_player->m_flMaxspeed() * 1.1f;
            if (max_speed > 0.f && speed > max_speed)
                data.m_velocity *= (max_speed / speed);
        }

        // Handle jump impulse
        if (data.m_was_in_air)
            data.m_velocity.z = g_csgo.sv_jump_impulse->GetFloat();
    }
    else {
        // Apply gravity when on ground
        data.m_velocity.z -= g_csgo.sv_gravity->GetFloat() * g_csgo.m_globals->m_interval;
    }

    // Perform collision checks with the environment
    CGameTrace trace{};
    CTraceFilterWorldOnly trace_filter{};
    g_csgo.m_engine_trace->TraceRay(
        { data.m_origin, data.m_origin + data.m_velocity * g_csgo.m_globals->m_interval, data.m_obb_min, data.m_obb_max },
        CONTENTS_SOLID, &trace_filter, &trace
    );

    // Handle collision response
    if (trace.m_fraction != 1.f) {
        for (int i = 0; i < 2; ++i) {
            data.m_velocity -= trace.m_plane.m_normal * data.m_velocity.dot(trace.m_plane.m_normal);
            float adjust = data.m_velocity.dot(trace.m_plane.m_normal);
            if (adjust < 0.f)
                data.m_velocity -= trace.m_plane.m_normal * adjust;

            g_csgo.m_engine_trace->TraceRay(
                { trace.m_endpos, trace.m_endpos + (data.m_velocity * (g_csgo.m_globals->m_interval * (1.f - trace.m_fraction))), data.m_obb_min, data.m_obb_max },
                CONTENTS_SOLID, &trace_filter, &trace
            );

            if (trace.m_fraction == 1.f)
                break;
        }
    }

    // Update final position and ground flag
    data.m_origin = trace.m_endpos;
    data.m_flags &= ~FL_ONGROUND;
    if (trace.m_fraction != 1.f && trace.m_plane.m_normal.z > 0.7f)
        data.m_flags |= FL_ONGROUND;
}
