#include "includes.h"

BoneHandler g_bone_handler{};;

__forceinline void m_AttachmentHelper(Entity* entity, CStudioHdr* hdr) {

    using AttachmentHelperFn = void(__thiscall*)(Entity*, CStudioHdr*);
    g_csgo.m_AttachmentHelper.as< AttachmentHelperFn  >()(entity, hdr);
}

bool BoneHandler::SetupBones( Player* player, BoneArray* pBoneMatrix, float time, bool disable_interp ) {
    player->m_bIsJiggleBonesEnabled() = false;

	const int effects = player->m_fEffects();
	const int lod_flags = player->m_iOcclusionFlags();
	const int anim_occlusion_frame_count = player->m_iOcclusionFramecount();
	const auto ik_ctx = player->m_pIKContext();
	const auto client_effects = player->m_ClientEntEffects();

	if( player != g_cl.m_local || disable_interp )
		player->m_fEffects() |= 8u;
	else
		player->m_fEffects() &= ~8u;

	player->m_iOcclusionFlags() &= ~2u;
	player->m_iOcclusionFramecount() = 0;
	player->m_flLastBoneSetupTime() = 0.f;
	player->RemoveIKContext();
	player->InvalidateBoneCache();
	// not sure of that one 
	// player->m_nCustomBlendingRuleMask() = -1;

	g_bone_handler.m_running = true;
	const bool result = player->SetupBones( pBoneMatrix, 128, 0x7FF00, time );
	g_bone_handler.m_running = false;

	player->m_pIKContext() = ik_ctx;
	player->m_fEffects() = effects;
	player->m_iOcclusionFlags() = lod_flags;
	player->m_iOcclusionFramecount() = anim_occlusion_frame_count;
	player->m_ClientEntEffects() = client_effects;

    return result;
}

bool BoneHandler::SetupBonesOnetap( Player* m_pPlayer, matrix3x4_t* m_pBones, bool m_bInterpolate )
{
	// backup vars.
	const int m_fBackupEffects = m_pPlayer->m_fEffects( );

	// backup globals
	const float m_flCurtime = g_csgo.m_globals->m_curtime;
	const float m_flRealTime = g_csgo.m_globals->m_realtime;
	const float m_flFrametime = g_csgo.m_globals->m_frametime;
	const float m_flAbsFrametime = g_csgo.m_globals->m_abs_frametime;
	const int m_iNextSimTick = m_pPlayer->m_flSimulationTime( ) / g_csgo.m_globals->m_interval + 1;
	const int m_nFrames = g_csgo.m_globals->m_frame;
	const int m_nTicks = g_csgo.m_globals->m_tick_count;

	// get jiggle bone cvar
	static ConVar* r_jiggle_bones = g_csgo.m_cvar->FindVar( HASH( "r_jiggle_bones" ) );

	// if jiggle bone isnt 0, force it to be 0
	if( r_jiggle_bones->GetInt( ) != 0 )
		r_jiggle_bones->SetValue( 0 );

	g_csgo.m_globals->m_curtime = m_pPlayer->m_flSimulationTime( );
	g_csgo.m_globals->m_realtime = m_pPlayer->m_flSimulationTime( );
	g_csgo.m_globals->m_frametime = g_csgo.m_globals->m_interval;
	g_csgo.m_globals->m_abs_frametime = g_csgo.m_globals->m_interval;
	g_csgo.m_globals->m_frame = m_iNextSimTick;
	g_csgo.m_globals->m_tick_count = m_iNextSimTick;

	// remove interpolation flag
	if ( !m_bInterpolate )
		m_pPlayer->m_fEffects( ) |= EF_NOINTERP;
	else
		m_pPlayer->m_fEffects( ) &= ~EF_NOINTERP;

	// setup bones
	const bool m_bRet = m_pPlayer->SetupBones( m_pBones, 128, BONE_USED_BY_ANYTHING, g_csgo.m_globals->m_curtime );
	
	// set back effects to their original state
	m_pPlayer->m_fEffects( ) = m_fBackupEffects;

	// restore globals.
	g_csgo.m_globals->m_curtime = m_flCurtime;
	g_csgo.m_globals->m_realtime = m_flRealTime;
	g_csgo.m_globals->m_frametime = m_flFrametime;
	g_csgo.m_globals->m_abs_frametime = m_flAbsFrametime;
	g_csgo.m_globals->m_frame = m_nFrames;
	g_csgo.m_globals->m_tick_count = m_nTicks;

	// return result
	return m_bRet;
}