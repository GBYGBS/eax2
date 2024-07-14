#include "includes.h"

void Hooks::RenderView(const CViewSetup& view, const CViewSetup& hud_view, int clear_flags, int what_to_draw) {
	// call og.
	g_hooks.m_view_render.GetOldMethod< OnRenderStart_t >(CViewRender::ONRENDERSTART)(this);

	static float stored_fov{ g_csgo.m_view_render->m_view.m_fov };
	float fov{ 90.f };

	//fov = g_config.i[XOR("effects_fov_value")];

	//if (g_cl.m_local && g_cl.m_local->m_bIsScoped()) {

	//	float percent = g_config.i[XOR("effects_fov_value_scope")] / 100.f;

	//	if (g_cl.m_weapon && g_cl.m_weapon->m_zoomLevel() != 2)
	//		fov = g_config.i[XOR("effects_fov_value")] / (1.f + 1.25f * percent);
	//	else
	//		fov = g_config.i[XOR("effects_fov_value")] / (1.f + 5.f * percent);
	//}

		if (stored_fov != fov)
			stored_fov = math::Lerp(g_csgo.m_globals->m_frametime * 16, stored_fov, fov);
	//}
	//else
	//	stored_fov = fov;

	g_csgo.m_view_render->m_view.m_fov = stored_fov;

	// g_cs'go.m_view_render->m_view.m_motionblur_mode = 2;
}

void Hooks::Render2DEffectsPostHUD(const CViewSetup& setup) {
	if (!g_menu.main.visuals.removals.get(3))
		g_hooks.m_view_render.GetOldMethod< Render2DEffectsPostHUD_t >(CViewRender::RENDER2DEFFECTSPOSTHUD)(this, setup);
}

void Hooks::RenderSmokeOverlay(bool unk) {
	// do not render smoke overlay.
	if (!g_menu.main.visuals.removals.get(1))
		g_hooks.m_view_render.GetOldMethod< RenderSmokeOverlay_t >(CViewRender::RENDERSMOKEOVERLAY)(this, unk);
}
