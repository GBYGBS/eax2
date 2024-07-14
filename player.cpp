#include "includes.h"

void Hooks::DoExtraBoneProcessing( int a2, int a3, int a4, int a5, int a6, int a7 ) {

	// dont call anything, skip leg interp etc..
	return;
}

void Hooks::BuildTransformations( int a2, int a3, int a4, int a5, int a6, int a7 ) {
	// cast thisptr to player ptr.
	Player* player = ( Player* )this;

	// get bone jiggle.
	int bone_jiggle = *reinterpret_cast< int* >( uintptr_t( player ) + 0x291C );

	// null bone jiggle to prevent attachments from jiggling around.
	*reinterpret_cast< int* >( uintptr_t( player ) + 0x291C ) = 0;

	// call og.
	g_hooks.m_BuildTransformations( this, a2, a3, a4, a5, a6, a7 );

	// restore bone jiggle.
	*reinterpret_cast< int* >( uintptr_t( player ) + 0x291C ) = bone_jiggle;
}

void Hooks::UpdateClientSideAnimation( ) {

	Player* player = ( Player* )this;

	// do nothing
	if( !g_csgo.m_engine->IsInGame( ) || !player || !g_cl.m_local )
		return g_hooks.m_UpdateClientSideAnimation( this );
	
	// if player isnt local
	if( g_cl.m_local != player ) {

		// woo
		if( !player->enemy( g_cl.m_local ) || !g_cl.m_processing || !g_menu.main.aimbot.enable.get( ) ) {

			// only update if their fakelag update
			if( player->m_flSimulationTime( ) > player->m_flOldSimulationTime( ) )
				g_hooks.m_UpdateClientSideAnimation( this );

			// exit
			return;
		}

		// if updating (cheat call)
		// NOTE: just adding this to make sure but shouldnt matter
		if( m_bUpdatingCSA )
			g_hooks.m_UpdateClientSideAnimation( this );

		// exit
		return;
	}

	// if local is dead
	if( !g_cl.m_processing )
		return g_hooks.m_UpdateClientSideAnimation( this );

	// run animation things here
	// g_cl.SetAngles( );
}

Weapon *Hooks::GetActiveWeapon( ) {
    Stack stack;

    static Address ret_1 = pattern::find( g_csgo.m_client_dll, XOR( "85 C0 74 1D 8B 88 ? ? ? ? 85 C9" ) );

    // note - dex; stop call to CIronSightController::RenderScopeEffect inside CViewRender::RenderView.
    if( g_menu.main.visuals.removals.get( 4 ) ) {
        if( stack.ReturnAddress( ) == ret_1 )
            return nullptr;
    }

    return g_hooks.m_GetActiveWeapon( this );
}

void Hooks::StandardBlendingRules( CStudioHdr* hdr, int a3, int a4, int a5, int mask ) {
	// cast thisptr to player ptr.
	Player* player = ( Player* )this;

	if( !player 
        || !player->IsPlayer( )
        || !player->alive( ) )
        return g_hooks.m_StandardBlendingRules( this, hdr, a3, a4, a5, mask );
	
	// backup effects.
	const int effects_backup = player->m_fEffects( );

	// store our current mask
	int bone_mask = mask;

	// check if we want to fix bones
	// set bone mask as bone used by server if non local
	if( player != g_cl.m_local )
		bone_mask = 0x3FD00;
	else // else add bone used by bone merge if local
		bone_mask |= 0x40000;

	// remove bone interpolation
	if( player != g_cl.m_local || !g_menu.main.misc.interpolation.get( ) )
		player->m_fEffects( ) |= EF_NOINTERP;

	// run original
	g_hooks.m_StandardBlendingRules( this, hdr, a3, a4, a5, bone_mask );

	// put our interpolation back to its previous state
	player->m_fEffects( ) = effects_backup;
}



void CustomEntityListener::OnEntityCreated( Entity *ent ) {
    if( ent ) {
        // player created.
        if( ent->IsPlayer( ) ) {
		    Player* player = ent->as< Player* >( );

		    // access out player data stucture and reset player data.
		    AimPlayer* data = &g_aimbot.m_players[ player->index( ) - 1 ];
            if( data )
		        data->reset( );
        }
	}
}

void CustomEntityListener::OnEntityDeleted( Entity *ent ) {
    // note; IsPlayer doesn't work here because the ent class is CBaseEntity.
	if( ent && ent->index( ) >= 1 && ent->index( ) <= 64 ) {
		Player* player = ent->as< Player* >( );

		// access out player data stucture and reset player data.
		AimPlayer* data = &g_aimbot.m_players[ player->index( ) - 1 ];
        if( data )
		    data->reset( );
	}
}