#include "includes.h"

int CSPlayerResource::GetPlayerPing(int idx) {
	static Address m_iPing = g_netvars.get(HASH("DT_PlayerResource"), HASH("m_iPing"));
	return *(int*)((uintptr_t)this + m_iPing + idx * 4);
}