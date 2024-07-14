#pragma once

class BoneHandler {
public:
    bool m_running;
    bool m_updating_anims;


public:

    bool SetupBones( Player* entity, BoneArray* pBoneMatrix, float time, bool disable_interp = false );
    bool SetupBonesOnetap( Player* m_pPlayer, matrix3x4_t* m_pBones, bool m_bInterpolate );
};

extern BoneHandler g_bone_handler;