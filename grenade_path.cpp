#include "includes.h"

void IEngineTrace::TraceLine(const vec3_t& src, const vec3_t& dst, int mask, IHandleEntity* entity, int collision_group, CGameTrace* trace) {
    static auto trace_filter_simple = pattern::find(g_csgo.m_client_dll, XOR("55 8B EC 83 E4 F0 83 EC 7C 56 52")) + 0x3D;

    std::uintptr_t filter[4] = { *reinterpret_cast<std::uintptr_t*>(trace_filter_simple), reinterpret_cast<std::uintptr_t>(entity), collision_group, 0 };

    TraceRay(Ray(src, dst), mask, reinterpret_cast<CTraceFilter*>(&filter), trace);
}

void IEngineTrace::TraceHull(const vec3_t& src, const vec3_t& dst, const vec3_t& mins, const vec3_t& maxs, int mask, IHandleEntity* entity, int collision_group, CGameTrace* trace) {
    static auto trace_filter_simple = pattern::find(g_csgo.m_client_dll, XOR("55 8B EC 83 E4 F0 83 EC 7C 56 52")) + 0x3D;

    std::uintptr_t filter[4] = { *reinterpret_cast<std::uintptr_t*>(trace_filter_simple), reinterpret_cast<std::uintptr_t>(entity), collision_group, 0 };

    TraceRay(Ray(src, dst, mins, maxs), mask, reinterpret_cast<CTraceFilter*>(&filter), trace);
}

void rotate_point(vec2_t& point, vec2_t origin, bool clockwise, float angle) {
    vec2_t delta = point - origin;
    vec2_t rotated;

    if (clockwise) {
        rotated = vec2_t(delta.x * cosf(angle) - delta.y * sinf(angle), delta.x * sinf(angle) + delta.y * cosf(angle));
    }
    else {
        rotated = vec2_t(delta.x * sinf(angle) - delta.y * cosf(angle), delta.x * cosf(angle) + delta.y * sinf(angle));
    }

    point = rotated + origin;
}

float& Entity::get_creation_time() {
    return *reinterpret_cast<float*>(0x29B0);
}

void c_grenade_prediction::on_create_move(CUserCmd* cmd) {
    m_data = {};

    if (!g_cl.m_processing || !g_menu.main.visuals.tracers.get())
        return;

    const auto weapon = reinterpret_cast<Weapon*>(g_csgo.m_entlist->GetClientEntityFromHandle(g_cl.m_local->GetActiveWeapon()));
    if (!weapon || !weapon->m_bPinPulled() && weapon->m_fThrowTime() == 0.f)
        return;

    const auto weapon_data = weapon->GetWpnData();
    if (!weapon_data || weapon_data->m_weapon_type != 9)
        return;

    m_data.m_owner = g_cl.m_local;
    m_data.m_index = weapon->m_iItemDefinitionIndex();

    auto view_angles = cmd->m_view_angles;

    if (view_angles.x < -90.f) {
        view_angles.x += 360.f;
    }
    else if (view_angles.x > 90.f) {
        view_angles.x -= 360.f;
    }

    view_angles.x -= (90.f - std::fabsf(view_angles.x)) * 10.f / 90.f;

    auto direction = vec3_t();

    math::AngleVectors(view_angles, direction);

    const auto throw_strength = std::clamp< float >(weapon->m_flThrowStrength(), 0.f, 1.f);
    const auto eye_pos = g_cl.m_shoot_pos;
    const auto src = vec3_t(eye_pos.x, eye_pos.y, eye_pos.z + (throw_strength * 12.f - 12.f));

    auto trace = CGameTrace();

    g_csgo.m_engine_trace->TraceHull(src, src + direction * 22.f, { -2.f, -2.f, -2.f }, { 2.f, 2.f, 2.f }, MASK_SOLID | CONTENTS_CURRENT_90, g_cl.m_local, COLLISION_GROUP_NONE, &trace);

    m_data.predict(trace.m_endpos - direction * 6.f, direction * (std::clamp< float >(weapon_data->m_throw_velocity * 0.9f, 15.f, 750.f) * (throw_strength * 0.7f + 0.3f)) + g_cl.m_local->m_vecVelocity() * 1.25f, g_csgo.m_globals->m_curtime, 0);
}

void DrawBeamPaw(vec3_t src, vec3_t end, Color color)
{
    BeamInfo_t beamInfo;
    beamInfo.m_nType = 0;
    beamInfo.m_nModelIndex = -1;
    beamInfo.m_flHaloScale = 0.f;
    beamInfo.m_flLife = 0.02f;
    beamInfo.m_flFadeLength = 10.f;
    beamInfo.m_flWidth = 2.f;
    beamInfo.m_flEndWidth = 2.f;
    beamInfo.m_pszModelName = "sprites/purplelaser1.vmt";
    beamInfo.m_flAmplitude = 0.f;
    beamInfo.m_flSpeed = 0.01f;
    beamInfo.m_nStartFrame = 0;
    beamInfo.m_flFrameRate = 0.f;
    beamInfo.m_flRed = color.r();
    beamInfo.m_flGreen = color.g();
    beamInfo.m_flBlue = color.b();
    beamInfo.m_flBrightness = color.a();
    beamInfo.m_nSegments = 2;
    beamInfo.m_bRenderable = true;
    beamInfo.m_nFlags = 0;
    beamInfo.m_vecStart = src;
    beamInfo.m_vecEnd = end;


    Beam_t* myBeam = g_csgo.m_beams->CreateBeamPoints(beamInfo);
    if (myBeam)
        g_csgo.m_beams->DrawBeam(myBeam);
}

bool c_grenade_prediction::data_t::draw() const
{
    if (!g_menu.main.visuals.grenade_path.get())
        return false;

    if (m_path.size() <= 1u || g_csgo.m_globals->m_curtime >= m_expire_time)
        return false;

    auto prev_screen = vec2_t();
    auto prev_on_screen = render::WorldToScreen(std::get< vec3_t >(m_path.front()), prev_screen);

    for (auto i = 1u; i < m_path.size(); ++i) {
        auto cur_screen = vec2_t();
        const auto cur_on_screen = render::WorldToScreen(std::get< vec3_t >(m_path.at(i)), cur_screen);

        if (prev_on_screen && cur_on_screen) {

            if (g_menu.main.visuals.grenade_path.get()) {

                float percent = ((m_expire_time - g_csgo.m_globals->m_curtime) / game::TICKS_TO_TIME(m_tick));

                Color retard = g_menu.main.visuals.grenade_path_col.get();

                DrawBeamPaw(std::get< vec3_t >(m_path.at(i - 1)), std::get< vec3_t >(m_path.at(i)), Color(retard.r(), retard.g(), retard.b(), retard.a() * percent));
            }
        }

        prev_screen = cur_screen;
        prev_on_screen = cur_on_screen;
    }
    return true;
}