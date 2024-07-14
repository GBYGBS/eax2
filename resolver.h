#pragma once
class Resolver {
public:

	enum Modes : size_t {
		RESOLVE_NONE = 0,
		RESOLVE_WALK,
		RESOLVE_LBY,
		RESOLVE_LBY_PRED,
		RESOLVE_STAND,
		RESOLVE_AIR,
		RESOLVE_STOPPED_MOVING,
		RESOLVE_NO_DATA,
		RESOLVE_DATA,
		RESOLVE_OVERRIDE,
		RESOLVE_NETWORK,
		RESOLVE_BODY_PRED,
		RESOLVE_SIDE_LASTMOVE,
		RESOLVE_REVERSEFS,
		RESOLVE_LASTMOVE,
		RESOLVE_BACK,
		RESOLVE_STAND2,
		RESOLVE_STAND1,
		RESOLVE_STAND3,
	};


public:
	LagRecord* FindIdealRecord(AimPlayer* data);
	LagRecord* FindLastRecord(AimPlayer* data);

	float AntiFreestand(Player* player, LagRecord* record, vec3_t start_, vec3_t end, bool include_base, float base_yaw, float delta);
	void ResolveOverride(AimPlayer* data, LagRecord* record, Player* player);

	void OnBodyUpdate(Player* player, float value);
	float GetAwayAngle(LagRecord* record);

	void MatchShot(AimPlayer* data, LagRecord* record);
	void SetMode(LagRecord* record);

	bool IsSideways(float angle, LagRecord* player);
	void ResolveAngles(Player* player, LagRecord* record);
	void ResolveAir(AimPlayer* data, LagRecord* record);
	void ResolveWalk(AimPlayer* data, LagRecord* record);
	int GetNearestEntity(Player* player, LagRecord* record);
	void ResolveStand(AimPlayer* data, LagRecord* record, Player* player);
	bool IsYawSideways(Player* entity, float yaw);
};

extern Resolver g_resolver;