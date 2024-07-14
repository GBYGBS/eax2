#include "includes.h"

Hooks                g_hooks{ };;
CustomEntityListener g_custom_entity_listener{ };;


#pragma optimize("", off)

void __fastcall secreto::modify_eye_pos( CCSGOPlayerAnimState* ecx, std::uintptr_t edx, vec3_t& pos) {
	// remove call from client
	return;
}

void __fastcall secreto::calc_view(void* ecx, const std::uintptr_t edx, vec3_t& eye_origin, const ang_t& eye_ang, float& z_near, float& z_far, float& fov) {
	if (!ecx || ecx != g_cl.m_local)
		return orig_calc_view(ecx, edx, eye_origin, eye_ang, z_near, z_far, fov);

	Player* player = (Player*)ecx;
	const bool backup = player->m_bUseNewAnimState();
	player->m_bUseNewAnimState() = false;
	orig_calc_view(ecx, edx, eye_origin, eye_ang, z_near, z_far, fov);
	player->m_bUseNewAnimState() = backup;
	return;
}

void __fastcall secreto::update_client_side_anim( Player* const player, const std::uintptr_t edx) {
	if (!player
		|| !player->alive()
		|| !player->networkable()
		|| player->index() < 1
		|| player->index() > 64)
		return orig_update_client_side_anim( player, edx );

	if( !g_cl.m_local ) {
		// g_cl.print("lol\n");
		return orig_update_client_side_anim(player, edx);
	}

	if( !g_cl.m_local->alive() ) {
		// g_cl.print("lol2\n");
		return orig_update_client_side_anim(player, edx);
	}
	// do nothing
	if( !g_csgo.m_engine->IsInGame( ) || !player )
		return orig_update_client_side_anim(player, edx );
	
	// if player isnt local
	if( player->index( ) != g_csgo.m_engine->GetLocalPlayer( ) || player != g_cl.m_local ) {

		// if updating (cheat call)
		// NOTE: just adding this to make sure but shouldnt matter
		if( g_hooks.m_bUpdatingCSA[ player->index() ] )
			orig_update_client_side_anim( player, edx );

		// exit
		return;
	}

	// if local is dead
	if( g_hooks.m_bUpdatingCSALP )
		return orig_update_client_side_anim( player, edx );

	g_cl.SetAngles();
	return;
}


void __fastcall secreto::do_extra_bones_processing( Player* const ecx, const std::uintptr_t edx, int a0, int a1, int a2, int a3, int a4, int a5) {
	return;
}

void __fastcall secreto::build_transformations( Player* ecx, void* edx, CStudioHdr* hdr, vec3_t* pos, quaternion_t* q, matrix3x4_t* cam_transform, int bone_mask, byte* computed )
{
	if (!ecx || ecx->index() < 1 || ecx->index() > 64)
		return orig_build_transformations(ecx, edx, hdr, pos, q, cam_transform, bone_mask, computed);
	
	const bool is_local = ecx->index( ) == g_csgo.m_engine->GetLocalPlayer( );
	const int effects = ecx->m_fEffects();

	if( !is_local )
		ecx->m_fEffects() |= 8;
	else {
		 if( !g_menu.main.misc.interpolation.get( ) )
			 ecx->m_fEffects() |= 8;
		 else
			 ecx->m_fEffects() &= ~8;
	}

	*reinterpret_cast< bool* >( uintptr_t( ecx ) + 0x291C ) = false;
	orig_build_transformations(ecx, edx, hdr, pos, q, cam_transform, bone_mask, computed);
	ecx->m_fEffects() = effects;
}


bool __fastcall secreto::should_skip_animation_frame(void* this_pointer, void* edx) {

	// the function is only called by SetupBones so there is no need to check for return address
	// returning false prevents copying of cached bone data

	return false;
}

void __fastcall secreto::check_for_sequence_change(void* this_pointer, void* edx, void* hdr, int cur_sequence, bool force_new_sequence, bool interpolate) {

	// no sequence interpolation over here mate
	// forces the animation queue to clear

	return orig_check_for_seq_change(this_pointer, edx, hdr, cur_sequence, force_new_sequence, false);
}


void __fastcall secreto::standard_blending_rules( Player* const ecx, const std::uintptr_t edx, CStudioHdr* const mdl_data, int a1, int a2, float a3, int mask) {
	if ( !ecx
		|| ecx->index() < 1
		|| ecx->index() > 64)
		return orig_standard_blending_rules(ecx, edx, mdl_data, a1, a2, a3, mask);


	const bool is_local = ecx->index( ) == g_csgo.m_engine->GetLocalPlayer( );
	const int effects = ecx->m_fEffects();
	int bone_mask = mask;

	if( !is_local ) {
		ecx->m_fEffects() |= 8;
		bone_mask = 0x3FD03; // bone used by server
	}
	else {
		 if( !g_menu.main.misc.interpolation.get( ) )
			 ecx->m_fEffects() |= 8;
		 else
			 ecx->m_fEffects() &= ~8;
	}

	orig_standard_blending_rules(ecx, edx, mdl_data, a1, a2, a3, bone_mask);
	ecx->m_fEffects() = effects;
}

void Body_proxy(CRecvProxyData* data, Address ptr, Address out) {
	Stack stack;

	static Address RecvTable_Decode{ pattern::find(g_csgo.m_engine_dll, XOR("EB 0D FF 77 10")) };

	// call from entity going into pvs.
	if (stack.next().next().ReturnAddress() != RecvTable_Decode) {
		// convert to player.
		Player* player = ptr.as< Player* >();

		// store data about the update.
		g_resolver.OnBodyUpdate(player, data->m_Value.m_Float);
	}

	// call original proxy.
	if (g_hooks.m_Body_original)
		g_hooks.m_Body_original(data, ptr, out);
}

void Force_proxy(CRecvProxyData* data, Address ptr, Address out) {
	// convert to ragdoll.
	Ragdoll* ragdoll = ptr.as< Ragdoll* >();

	// get ragdoll owner.
	Player* player = ragdoll->GetPlayer();

	// we only want this happening to noobs we kill.
	if (g_cl.m_local && player && player->enemy(g_cl.m_local)) {
		// get m_vecForce.
		vec3_t vel = { data->m_Value.m_Vector[0], data->m_Value.m_Vector[1], data->m_Value.m_Vector[2] };

		// give some speed to all directions.
		vel *= 1000.f;

		// boost z up a bit.
		if (vel.z <= 1.f)
			vel.z = 2.f;

		vel.z *= 2.f;

		// don't want crazy values for this... probably unlikely though?
		math::clamp(vel.x, std::numeric_limits< float >::lowest(), std::numeric_limits< float >::max());
		math::clamp(vel.y, std::numeric_limits< float >::lowest(), std::numeric_limits< float >::max());
		math::clamp(vel.z, std::numeric_limits< float >::lowest(), std::numeric_limits< float >::max());

		// set new velocity.
		data->m_Value.m_Vector[0] = vel.x;
		data->m_Value.m_Vector[1] = vel.y;
		data->m_Value.m_Vector[2] = vel.z;
	}

	if (g_hooks.m_Force_original)
		g_hooks.m_Force_original(data, ptr, out);
}


#define hook( target, hook, original ) \
   MH_CreateHook( Address{ target }.as< LPVOID >( ), \
        reinterpret_cast< LPVOID >( &hook ), reinterpret_cast< LPVOID* >( &original ) )

void Hooks::init( ) {
	// hook wndproc.
	m_old_wndproc = ( WNDPROC )g_winapi.SetWindowLongA( g_csgo.m_game->m_hWindow, GWL_WNDPROC, util::force_cast< LONG >( Hooks::WndProc ) );

	// setup normal VMT hooks.
	m_panel.init( g_csgo.m_panel );
	m_panel.add( IPanel::PAINTTRAVERSE, util::force_cast( &Hooks::PaintTraverse ) );

	m_client.init( g_csgo.m_client );
	m_client.add( CHLClient::LEVELINITPREENTITY, util::force_cast( &Hooks::LevelInitPreEntity ) );
	m_client.add( CHLClient::LEVELINITPOSTENTITY, util::force_cast( &Hooks::LevelInitPostEntity ) );
	m_client.add( CHLClient::LEVELSHUTDOWN, util::force_cast( &Hooks::LevelShutdown ) );
	m_client.add( CHLClient::FRAMESTAGENOTIFY, util::force_cast( &Hooks::FrameStageNotify ) );

	m_engine.init( g_csgo.m_engine );
	m_engine.add( IVEngineClient::ISCONNECTED, util::force_cast( &Hooks::IsConnected ) );
	m_engine.add( IVEngineClient::ISHLTV, util::force_cast( &Hooks::IsHLTV ) );
	m_engine.add(IVEngineClient::ISPAUSED, util::force_cast(&Hooks::IsPaused));

	m_prediction.init( g_csgo.m_prediction );
	m_prediction.add( CPrediction::INPREDICTION, util::force_cast( &Hooks::InPrediction ) );
	m_prediction.add( CPrediction::RUNCOMMAND, util::force_cast( &Hooks::RunCommand ) );

	m_client_mode.init( g_csgo.m_client_mode );
	m_client_mode.add( IClientMode::SHOULDDRAWFOG, util::force_cast( &Hooks::ShouldDrawFog ) );
	m_client_mode.add( IClientMode::OVERRIDEVIEW, util::force_cast( &Hooks::OverrideView ) );
	m_client_mode.add( IClientMode::CREATEMOVE, util::force_cast( &Hooks::CreateMove ) );
	m_client_mode.add( IClientMode::DOPOSTSPACESCREENEFFECTS, util::force_cast( &Hooks::DoPostScreenSpaceEffects ) );

	m_surface.init( g_csgo.m_surface );
	m_surface.add( ISurface::LOCKCURSOR, util::force_cast( &Hooks::LockCursor ) );
	m_surface.add( ISurface::PLAYSOUND, util::force_cast( &Hooks::PlaySound ) );
	m_surface.add( ISurface::ONSCREENSIZECHANGED, util::force_cast( &Hooks::OnScreenSizeChanged ) );

	m_model_render.init( g_csgo.m_model_render );
	m_model_render.add( IVModelRender::DRAWMODELEXECUTE, util::force_cast( &Hooks::DrawModelExecute ) );

	m_render_view.init( g_csgo.m_render_view );
	m_render_view.add( IVRenderView::SCENEEND, util::force_cast( &Hooks::SceneEnd ) );

	m_shadow_mgr.init( g_csgo.m_shadow_mgr );
	m_shadow_mgr.add( IClientShadowMgr::COMPUTESHADOWDEPTHTEXTURES, util::force_cast( &Hooks::ComputeShadowDepthTextures ) );

	m_view_render.init( g_csgo.m_view_render );
	m_view_render.add( CViewRender::RENDER2DEFFECTSPOSTHUD, util::force_cast( &Hooks::Render2DEffectsPostHUD ) );
	m_view_render.add( CViewRender::RENDERSMOKEOVERLAY, util::force_cast( &Hooks::RenderSmokeOverlay ) );

	m_material_system.init( g_csgo.m_material_system );
	m_material_system.add( IMaterialSystem::OVERRIDECONFIG, util::force_cast( &Hooks::OverrideConfig ) );

	m_client_state.init( g_csgo.m_hookable_cl );
	m_client_state.add( CClientState::TEMPENTITIES, util::force_cast( &Hooks::TempEntities ) );
	// m_client_state.add( CClientState::PACKETSTART, util::force_cast( &Hooks::PacketStart ) );
	m_client_state.add( CClientState::VOICEDATA, util::force_cast( &Hooks::hkVoiceData ) );

	// register our custom entity listener.
	// todo - dex; should we push our listeners first? should be fine like this.
	g_custom_entity_listener.init( );

	// cvar hooks.
	m_debug_spread.init( g_csgo.weapon_debug_spread_show );
	m_debug_spread.add( ConVar::GETINT, util::force_cast( &Hooks::DebugSpreadGetInt ) );

	m_net_show_fragments.init( g_csgo.net_showfragments );
	m_net_show_fragments.add( ConVar::GETBOOL, util::force_cast( &Hooks::NetShowFragmentsGetBool ) );

	// set netvar proxies.
	g_netvars.SetProxy(HASH("DT_CSPlayer"), HASH("m_body"), Body_proxy, m_Body_original);
	g_netvars.SetProxy(HASH("DT_CSRagdoll"), HASH("m_vecForce"), Force_proxy, m_Force_original);


	static auto modif_eye_pos_addr = pattern::find(g_csgo.m_client_dll, "55 8B EC 83 E4 F8 83 EC 58 56 57 8B F9 83 7F 60 00");
	static auto calc_view_addr = pattern::find(g_csgo.m_client_dll, "55 8B EC 53 8B 5D 08 56 57 FF 75 18");
	static auto update_csa_addr = pattern::find(g_csgo.m_client_dll, "55 8B EC 51 56 8B F1 80 BE ? ? ? ? ? 74 36");
	static auto do_extra_bone_proc_addr = pattern::find(g_csgo.m_client_dll, "55 8B EC 83 E4 F8 81 EC ? ? ? ? 53 56 8B F1 57 89 74 24 1C");
	static auto build_trans_addr = pattern::find(g_csgo.m_client_dll, "55 8B EC 83 E4 F0 81 EC ? ? ? ? 56 57 8B F9 8B 0D ? ? ? ? 89 7C 24 1C");
	static auto check_seq_change_addr = pattern::find(g_csgo.m_client_dll, "55 8B EC 51 53 8B 5D 08 56 8B F1 57 85");
	static auto skip_frame_addr = pattern::find(g_csgo.m_client_dll, "57 8B F9 8B 07 8B 80 ? ? ? ? FF D0 84 C0 75 02");
	static auto standard_blending_rules_addr = pattern::find(g_csgo.m_client_dll, "55 8B EC 83 E4 F0 B8 ? ? ? ? E8 ? ? ? ? 56 8B 75 08 57 8B F9 85 F6");

	MH_Initialize();
	// MH_CreateHook( pattern::find( PE::GetModule( HASH( "engine.dll" ) ), XOR( "55 8B EC 56 8B F1 8B 86 ? ? ? ? 85 C0 74 24 48 83 F8 02 77 2C 83 BE ? ? ? ? ? 8D 8E ? ? ? ? 74 06 32 C0 84 C0 EB 10 E8 ? ? ? ? 84 C0 EB 07 83 BE ? ? ? ? ? 0F 94 C0 84 C0 74 07 B0 01 5E 5D C2 0C 00" ) ), &nem_hooks::SendNetMsg, reinterpret_cast< void** >( &nem_hooks::oSendNetMsg ) );

	hook( modif_eye_pos_addr,           secreto::modify_eye_pos, secreto::orig_modify_eye_pos);
	hook( calc_view_addr,				secreto::calc_view, secreto::orig_calc_view);
	hook( update_csa_addr,				secreto::update_client_side_anim, secreto::orig_update_client_side_anim);
	hook( do_extra_bone_proc_addr,		secreto::do_extra_bones_processing, secreto::orig_do_extra_bones_processing);
	hook( build_trans_addr,				secreto::build_transformations, secreto::orig_build_transformations);
	hook( check_seq_change_addr,		secreto::check_for_sequence_change, secreto::orig_check_for_seq_change);
	hook( skip_frame_addr,				secreto::should_skip_animation_frame, secreto::orig_should_skip_animation_frame);
	hook( standard_blending_rules_addr, secreto::standard_blending_rules, secreto::orig_standard_blending_rules);
	MH_CreateHook(pattern::find(PE::GetModule(HASH("engine.dll")), XOR("55 8B EC 56 8B F1 8B 86 ? ? ? ? 85 C0 74 24 48 83 F8 02 77 2C 83 BE ? ? ? ? ? 8D 8E ? ? ? ? 74 06 32 C0 84 C0 EB 10 E8 ? ? ? ? 84 C0 EB 07 83 BE ? ? ? ? ? 0F 94 C0 84 C0 74 07 B0 01 5E 5D C2 0C 00")), &nem_hooks::SendNetMsg, reinterpret_cast<void**>(&nem_hooks::oSendNetMsg));

	MH_EnableHook(MH_ALL_HOOKS);


}


#pragma optimize("", on)