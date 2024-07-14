#include "includes.h"







struct CIncomingSequence {
	int InSequence;
	int ReliableState;
};

std::vector<CIncomingSequence> IncomingSequences;

int lastsent = 0;
bool __fastcall nem_hooks::SendNetMsg(INetChannel* pNetChan, void* edx, INetMessage& msg, bool bForceReliable, bool bVoice) {
	if (pNetChan != g_csgo.m_engine->GetNetChannelInfo())
		return oSendNetMsg(pNetChan, msg, bForceReliable, bVoice);

	if (msg.GetType() == 14) // Return and don't send messsage if its FileCRCCheck
		return false;

	if (msg.GetGroup() == 11) {

		constexpr int EXPIRE_DURATION = 2500; // miliseconds-ish?
		bool should_send = GetTickCount() - lastsent > EXPIRE_DURATION;
		if (should_send) {
			Voice_Vader packet;
			strcpy(packet.cheat_name, XOR("21372115")); // old wNeJbkUsTleR
			packet.make_sure = 1;
			VoiceDataCustom data;
			memcpy(data.get_raw_data(), &packet, sizeof(packet));

			CCLCMsg_VoiceData_Legacy msg;
			memset(&msg, 0, sizeof(msg));

			//static DWORD m_construct_voice_message = ( DWORD )Memory::Scan( ( "engine.dll" ), ( "56 57 8B F9 8D 4F 08 C7 07 ? ? ? ? E8 ? ? ? ? C7" ) );

			auto func = (uint32_t(__fastcall*)(void*, void*))g_csgo.m_construct_voice_message;
			func((void*)&msg, nullptr);

			// set our data
			msg.set_data(&data);

			// mad!
			lame_string_t lame_string;

			// set rest
			msg.data = &lame_string;
			msg.format = 0; // VoiceFormat_Steam
			msg.flags = 63; // all flags!

			// send it
			oSendNetMsg(pNetChan, (INetMessage&)msg, false, true);
			lastsent = GetTickCount();
		}
	}
	else if (msg.GetGroup() == 9) { // group 9 is VoiceData
		// Fixing fakelag with voice
		bVoice = true;
		g_cl.m_enable_voice = true;
	}
	else
		g_cl.m_enable_voice = false;

	return oSendNetMsg(pNetChan, msg, bForceReliable, bVoice);
}