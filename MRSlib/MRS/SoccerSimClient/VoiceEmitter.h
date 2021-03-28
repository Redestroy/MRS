#pragma once
#include <MRS/Include/CommunicationDevice.hpp>
#include "UDPController.h"
#include "SoccerServerCommandDict.h"
#include <deque>
#include <map>
namespace MRS {
	namespace Device {
		class VoiceEmitter_UDPController : public CommunicationDevice
		{
		private:
			UDPController* udpController_ptr;
			SoccerServerCommandDict* cmd_dict;
			std::deque<std::string> args_say, args_hear;
		public:
			VoiceEmitter_UDPController();
			~VoiceEmitter_UDPController();

			void SendMessage(std::string message);
			std::string GetPacket();

			void SetCommandDict(SoccerServerCommandDict*);
			SoccerServerCommandDict* GetCommandDict();
			void SetUDPController(UDPController*);
			UDPController* GetUDPController();
		};
	}
}
