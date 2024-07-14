#include "includes.h"

#define NET_FRAMES_BACKUP 64 // must be power of 2. 
#define NET_FRAMES_MASK ( NET_FRAMES_BACKUP - 1 )

int Hooks::SendDatagram(void* data) {
    int backup1 = g_csgo.m_net->m_in_rel_state;
    int backup2 = g_csgo.m_net->m_in_seq;

    if (g_aimbot.m_fake_latency || g_aimbot.m_fake_latency2) {
        float ping = g_aimbot.m_fake_latency2 ? g_menu.main.misc.secondary_fake_latency_amt.get() : g_menu.main.misc.fake_latency_amt.get();

        // the target latency.
        float correct = game::TICKS_TO_TIME(game::TIME_TO_TICKS(std::clamp(ping / 1000.f, 0.f, g_csgo.sv_maxunlag->GetFloat()) - g_cl.m_lerp));

        if (correct >= g_csgo.m_globals->m_interval)
        {
            // iterate sequences.
            for (auto& s : g_cl.m_sequences) {
                float delta = g_csgo.m_globals->m_realtime - s.m_time;

                if (delta >= correct) {
                    g_csgo.m_net->m_in_rel_state = s.m_state;
                    g_csgo.m_net->m_in_seq = s.m_seq;
                    break;
                }
            }
        }
    }

    int ret = g_hooks.m_net_channel.GetOldMethod< SendDatagram_t >(INetChannel::SENDDATAGRAM)(this, data);

    g_csgo.m_net->m_in_rel_state = backup1;
    g_csgo.m_net->m_in_seq = backup2;

    return ret;
}

void Hooks::ProcessPacket(void* packet, bool header) {
	g_hooks.m_net_channel.GetOldMethod< ProcessPacket_t >(INetChannel::PROCESSPACKET)(this, packet, header);

	g_cl.UpdateIncomingSequences();

	// get this from CL_FireEvents string "Failed to execute event for classId" in engine.dll
	for (CEventInfo* it{ g_csgo.m_cl->m_events }; it != nullptr; it = it->m_next) {
		if (!it->m_class_id)
			continue;

		// set all delays to instant.
		it->m_fire_delay = 0.f;
	}

	// game events are actually fired in OnRenderStart which is WAY later after they are received
	// effective delay by lerp time, now we call them right after theyre received (all receive proxies are invoked without delay).
	g_csgo.m_engine->FireEvents();
}