#include "VoiceEmitter.h"
namespace MRS {
	namespace Device {
		VoiceEmitter_UDPController::VoiceEmitter_UDPController()
		{

		}

		VoiceEmitter_UDPController::~VoiceEmitter_UDPController()
		{
		}

		void VoiceEmitter_UDPController::SendMessage(std::string message)
		{
			//send command say
			args_say.clear();
			args_say.emplace_back("\"" + message + "\""); //team name
			udpController_ptr->SendCommand(cmd_dict->GetCommandTemplate("say"), cmd_dict->ParseArgsIntoTemplate((cmd_dict->GetArgsTemplate("say")), args_say));
			//std::string init_response = udpController_ptr->GetPacket(); //Send Init command
			//std::cout << "Init response: " << init_response << "\n";
		}

		std::string VoiceEmitter_UDPController::GetPacket()
		{
			//Send command hear
			args_hear.clear();
			//args_hear.emplace_back("");
			udpController_ptr->SendCommand(cmd_dict->GetCommandTemplate("move"), cmd_dict->ParseArgsIntoTemplate((cmd_dict->GetArgsTemplate("move")), args_hear));
			std::string response = udpController_ptr->GetPacket(); //Send Init command
			std::map<std::string, std::string> response_map = cmd_dict->ParseResponse_Hear(response);
			std::string response_str = response_map["payload"];
			return response_str;
		}

		void VoiceEmitter_UDPController::SetCommandDict(SoccerServerCommandDict* cmd_dict)
		{
			this->cmd_dict = cmd_dict;
		}

		SoccerServerCommandDict* VoiceEmitter_UDPController::GetCommandDict()
		{
			return cmd_dict;
		}

		void VoiceEmitter_UDPController::SetUDPController(UDPController* ptr)
		{
			this->udpController_ptr = ptr;
		}

		UDPController* VoiceEmitter_UDPController::GetUDPController()
		{
			return udpController_ptr;
		}
	}
}