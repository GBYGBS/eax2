#pragma once


struct Voice_Vader
{
	char cheat_name[13];
	int make_sure;
	const char* username;
};


struct VoiceDataCustom
{
	uint32_t xuid_low{};
	uint32_t xuid_high{};
	int32_t sequence_bytes{};
	uint32_t section_number{};
	uint32_t uncompressed_sample_offset{};

	__forceinline uint8_t* get_raw_data()
	{
		return (uint8_t*)this;
	}
};

struct CCLCMsg_VoiceData_Legacy
{
	uint32_t INetMessage_Vtable; //0x0000
	char pad_0004[4]; //0x0004
	uint32_t CCLCMsg_VoiceData_Vtable; //0x0008
	char pad_000C[8]; //0x000C
	void* data; //0x0014
	uint32_t xuid_low{};
	uint32_t xuid_high{};
	int32_t format; //0x0020
	int32_t sequence_bytes; //0x0024
	uint32_t section_number; //0x0028
	uint32_t uncompressed_sample_offset; //0x002C
	int32_t cached_size; //0x0030

	uint32_t flags; //0x0034

	uint8_t no_stack_overflow[0xFF];

	__forceinline void set_data(VoiceDataCustom* cdata)
	{
		xuid_low = cdata->xuid_low;
		xuid_high = cdata->xuid_high;
		sequence_bytes = cdata->sequence_bytes;
		section_number = cdata->section_number;
		uncompressed_sample_offset = cdata->uncompressed_sample_offset;
	}
};

struct CSVCMsg_VoiceData_Legacy
{
	char pad_0000[8]; //0x0000
	int32_t client; //0x0008
	int32_t audible_mask; //0x000C
	uint32_t xuid_low{};
	uint32_t xuid_high{};
	void* voide_data_; //0x0018
	int32_t proximity; //0x001C
	//int32_t caster; //0x0020
	int32_t format; //0x0020
	int32_t sequence_bytes; //0x0024
	uint32_t section_number; //0x0028
	uint32_t uncompressed_sample_offset; //0x002C

	__forceinline VoiceDataCustom get_data()
	{
		VoiceDataCustom cdata;
		cdata.xuid_low = xuid_low;
		cdata.xuid_high = xuid_high;
		cdata.sequence_bytes = sequence_bytes;
		cdata.section_number = section_number;
		cdata.uncompressed_sample_offset = uncompressed_sample_offset;
		return cdata;
	}
};

struct lame_string_t

{
	char data[16]{};
	uint32_t current_len = 0;
	uint32_t max_len = 15;
};

namespace nem_hooks {
	using SendNetMsgFn = bool(__thiscall*)(INetChannel* pNetChan, INetMessage& msg, bool bForceReliable, bool bVoice);
	inline SendNetMsgFn oSendNetMsg;
	bool __fastcall SendNetMsg(INetChannel* pNetChan, void* edx, INetMessage& msg, bool bForceReliable, bool bVoice);

}

class Hooks {
public:
	void init( );

public:
	// forward declarations
	class IRecipientFilter;

	// prototypes.
	using PaintTraverse_t              = void( __thiscall* )( void*, VPANEL, bool, bool );
	using DoPostScreenSpaceEffects_t   = bool( __thiscall* )( void*, CViewSetup* );
	using CreateMove_t                 = bool( __thiscall* )( void*, float, CUserCmd* );
	using LevelInitPostEntity_t        = void( __thiscall* )( void* );
	using LevelShutdown_t              = void( __thiscall* )( void* );
	using LevelInitPreEntity_t         = void( __thiscall* )( void*, const char* );
	using IN_KeyEvent_t                = int( __thiscall* )( void*, int, int, const char* );
	using FrameStageNotify_t           = void( __thiscall* )( void*, Stage_t );
	using UpdateClientSideAnimation_t  = void( __thiscall* )( void* );
    using GetActiveWeapon_t            = Weapon*( __thiscall * )( void* );
	using DoExtraBoneProcessing_t      = void( __thiscall* )( void*, int, int, int, int, int, int );
	using StandardBlendingRules_t = void(__thiscall*)(void*, CStudioHdr*, int, int, int, int);
	using CalcView_t = void(__thiscall*)(void*, vec3_t&, vec3_t&, float&, float&, float&);
	using BuildTransformations_t       = void( __thiscall* )( void*, int, int, int, int, int, int );
	using CalcViewModelView_t          = void( __thiscall* )( void*, vec3_t&, ang_t& );
	using InPrediction_t               = bool( __thiscall* )( void* );
	using OverrideView_t               = void( __thiscall* )( void*, CViewSetup* );
	using LockCursor_t                 = void( __thiscall* )( void* );
	using RunCommand_t                 = void( __thiscall* )( void*, Entity*, CUserCmd*, IMoveHelper* );
	using ProcessPacket_t              = void( __thiscall* )( void*, void*, bool );
	using SendDatagram_t               = int( __thiscall* )( void*, void* );
	// using CanPacket_t                = bool( __thiscall* )( void* );
	using PlaySound_t                  = void( __thiscall* )( void*, const char* );
	using GetScreenSize_t              = void( __thiscall* )( void*, int&, int& );
	using Push3DView_t                 = void( __thiscall* )( void*, CViewSetup&, int, void*, void* );
	using SceneEnd_t                   = void( __thiscall* )( void* );
	using DrawModelExecute_t           = void( __thiscall* )( void*, uintptr_t, const DrawModelState_t&, const ModelRenderInfo_t&, matrix3x4_t* );
	using ComputeShadowDepthTextures_t = void( __thiscall* )( void*, const CViewSetup&, bool );
	using GetInt_t                     = int( __thiscall* )( void* );
	using GetBool_t                    = bool( __thiscall* )( void* );
	using IsConnected_t                = bool( __thiscall* )( void* );
	using IsHLTV_t                     = bool( __thiscall* )( void* );
	using IsPaused_t = bool(__thiscall*)(void*);
	using OnEntityCreated_t            = void( __thiscall* )( void*, Entity* );
	using OnEntityDeleted_t            = void( __thiscall* )( void*, Entity* );
	using RenderSmokeOverlay_t         = void( __thiscall* )( void*, bool );
	using ShouldDrawFog_t              = bool( __thiscall* )( void* );
	using ShouldDrawParticles_t        = bool( __thiscall* )( void* );
	using Render2DEffectsPostHUD_t     = void( __thiscall* )( void*, const CViewSetup& );
	using OnRenderStart_t              = void( __thiscall* )( void* );
    using RenderView_t                 = void( __thiscall* )( void*, const CViewSetup &, const CViewSetup &, int, int );
	using GetMatchSession_t            = CMatchSessionOnlineHost*( __thiscall* )( void* );
	using OnScreenSizeChanged_t        = void( __thiscall* )( void*, int, int );
	using OverrideConfig_t             = bool( __thiscall* )( void*, MaterialSystem_Config_t*, bool );
	using PostDataUpdate_t             = void( __thiscall* )( void*, DataUpdateType_t );
	using TempEntities_t               = bool( __thiscall* )( void*, void * );
	using PacketStart_t = void(__thiscall*)(void*, int, int);
	using EmitSound_t                  = void( __thiscall* )( void*, IRecipientFilter&, int, int, const char*, unsigned int, const char*, float, float, int, int, int, const vec3_t*, const vec3_t*, void*, bool, float, int );
	// using PreDataUpdate_t            = void( __thiscall* )( void*, DataUpdateType_t );
	using FnVoiceData = void(__thiscall*)(void*, void*);
	using BeginFrameFn = void(__thiscall*)(void*, float);

public:
	bool                     TempEntities( void *msg );
	void                     PacketStart(int incoming_sequence, int outgoing_acknowledged);
	void __fastcall          hkVoiceData(void* msg);
	void                     PaintTraverse( VPANEL panel, bool repaint, bool force );
	bool                     DoPostScreenSpaceEffects( CViewSetup* setup );
	bool                     CreateMove( float input_sample_time, CUserCmd* cmd );
	void                     LevelInitPostEntity( );
	void                     LevelShutdown( );
	//int                      IN_KeyEvent( int event, int key, const char* bind );
	void                     LevelInitPreEntity( const char* map );
	void                     FrameStageNotify( Stage_t stage );
	void                     UpdateClientSideAnimation( );
    Weapon*                  GetActiveWeapon( );
	bool                     InPrediction( );
	bool                     ShouldDrawParticles( );
	bool                     ShouldDrawFog( );
	void                     OverrideView( CViewSetup* view );
	void                     LockCursor( );
	void                     PlaySound( const char* name );
	void                     OnScreenSizeChanged( int oldwidth, int oldheight );
	void                     RunCommand( Entity* ent, CUserCmd* cmd, IMoveHelper* movehelper );
	int                      SendDatagram( void* data );
	void                     ProcessPacket( void* packet, bool header );
	//void                     GetScreenSize( int& w, int& h );
	void                     SceneEnd( );
	void                     DrawModelExecute( uintptr_t ctx, const DrawModelState_t& state, const ModelRenderInfo_t& info, matrix3x4_t* bone );
	void                     ComputeShadowDepthTextures( const CViewSetup& view, bool unk );
	int                      DebugSpreadGetInt( );
	bool                     NetShowFragmentsGetBool( );
	void                     DoExtraBoneProcessing( int a2, int a3, int a4, int a5, int a6, int a7 );
	void StandardBlendingRules(CStudioHdr* hdr, int a3, int a4, int a5, int mask);
	void CalcView(vec3_t& eye_origin, vec3_t& eye_angles, float& z_near, float& z_far, float& fov);
	void                     BuildTransformations( int a2, int a3, int a4, int a5, int a6, int a7 );
	bool                     IsConnected( );
	bool                     IsHLTV( );
	bool IsPaused();
	void                     EmitSound( IRecipientFilter& filter, int iEntIndex, int iChannel, const char* pSoundEntry, unsigned int nSoundEntryHash, const char* pSample, float flVolume, float flAttenuation, int nSeed, int iFlags, int iPitch, const vec3_t* pOrigin, const vec3_t* pDirection, void* pUtlVecOrigins, bool bUpdatePositions, float soundtime, int speakerentity );
	void                     RenderSmokeOverlay( bool unk );
    void                     RenderView( const CViewSetup &view, const CViewSetup &hud_view, int clear_flags, int what_to_draw );
	void                     Render2DEffectsPostHUD( const CViewSetup& setup );
	CMatchSessionOnlineHost* GetMatchSession( );
	bool                     OverrideConfig( MaterialSystem_Config_t* config, bool update );
	void __fastcall                     BeginFrame(float ft);
	void                     PostDataUpdate( DataUpdateType_t type );

	static LRESULT WINAPI WndProc( HWND wnd, uint32_t msg, WPARAM wp, LPARAM lp );

public:
	// vmts.
	VMT m_panel;
	VMT m_client_mode;
	VMT m_client;
	VMT m_client_state;
	VMT m_engine;
	VMT m_engine_sound;
	VMT m_prediction;
	VMT m_surface;
	VMT m_render;
	VMT m_net_channel;
	VMT m_render_view;
	VMT m_model_render;
	VMT m_shadow_mgr;
	VMT m_view_render;
	VMT m_match_framework;
	VMT m_material_system;
	VMT m_fire_bullets;
	VMT m_net_show_fragments;

	// cvars
	VMT m_debug_spread;

	// wndproc old ptr.
	WNDPROC m_old_wndproc;

	// old player create fn.
	DoExtraBoneProcessing_t     m_DoExtraBoneProcessing;
	UpdateClientSideAnimation_t m_UpdateClientSideAnimation;
    GetActiveWeapon_t           m_GetActiveWeapon;
	BuildTransformations_t      m_BuildTransformations;
	StandardBlendingRules_t		m_StandardBlendingRules;
	CalcView_t                  m_CalcView;



	bool m_bUpdatingCSA[65];
	bool m_bUpdatingCSALP;

	// netvar proxies.
	RecvVarProxy_t m_Pitch_original;
	RecvVarProxy_t m_Yaw_original;
	RecvVarProxy_t m_Body_original;
	RecvVarProxy_t m_Force_original;
	RecvVarProxy_t m_AbsYaw_original;
	RecvVarProxy_t m_SimTime_original;
};

// note - dex; these are defined in player.cpp.
class CustomEntityListener : public IEntityListener {
public:
    virtual void OnEntityCreated( Entity *ent );
    virtual void OnEntityDeleted( Entity *ent );

    __forceinline void init( ) {
        g_csgo.AddListenerEntity( this );
    }
};


namespace secreto {
	void __fastcall modify_eye_pos( CCSGOPlayerAnimState* ecx, std::uintptr_t edx, vec3_t& pos);
	inline decltype(&modify_eye_pos) orig_modify_eye_pos{ };


	void __fastcall calc_view(void* ecx, const std::uintptr_t edx, vec3_t& eye_origin, const ang_t& eye_ang, float& z_near, float& z_far, float& fov);
	inline decltype(&calc_view) orig_calc_view{ };

	void __fastcall update_client_side_anim( Player* const player, const std::uintptr_t edx);
	inline decltype(&update_client_side_anim) orig_update_client_side_anim{ };

	void __fastcall do_extra_bones_processing( Player* const ecx, const std::uintptr_t edx, int a0, int a1, int a2, int a3, int a4, int a5 );
	inline decltype(&do_extra_bones_processing) orig_do_extra_bones_processing{ };

	void __fastcall build_transformations(
		Player* ecx, void* edx, CStudioHdr* hdr, vec3_t* pos, quaternion_t* q, matrix3x4_t* cam_transform, int bone_mask, byte* computed
	);
	inline decltype(&build_transformations) orig_build_transformations{ };

	bool __fastcall should_skip_animation_frame(void* this_pointer, void* edx);
	inline decltype(&should_skip_animation_frame) orig_should_skip_animation_frame{ };

	void __fastcall check_for_sequence_change(void* this_pointer, void* edx, void* hdr, int cur_sequence, bool force_new_sequence, bool interpolate);
	inline decltype(&check_for_sequence_change) orig_check_for_seq_change{ };

	void __fastcall standard_blending_rules( Player* const ecx, const std::uintptr_t edx, CStudioHdr* const mdl_data, int a1, int a2, float a3, int mask );
	inline decltype(&standard_blending_rules) orig_standard_blending_rules{ };

};


extern Hooks                g_hooks;
extern CustomEntityListener g_custom_entity_listener;