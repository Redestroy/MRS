#include "MovementActuator2D_UDPController.h"
namespace MRS {
	namespace Device {
		MovementActuator2D_UDPController::MovementActuator2D_UDPController() : args_dash(), args_move()
		{
			udpController_ptr = nullptr;
		}

		MovementActuator2D_UDPController::MovementActuator2D_UDPController(UDPController* ptr): args_dash(), args_move()
		{
			udpController_ptr = ptr;
		}

		MovementActuator2D_UDPController::MovementActuator2D_UDPController(UDPController* ptr, std::string cmd_format) : MovementActuator2D_UDPController(ptr)
		{
			//TODO: replace with ProtocolParser method in the future
			command_template = cmd_format.substr(0, cmd_format.find(';'));
			args_template = cmd_format.substr(cmd_format.find(';') + 1, cmd_format.find('\r') - cmd_format.find(';') - 1);
		}

		MovementActuator2D_UDPController::~MovementActuator2D_UDPController()
		{
		}

		void MovementActuator2D_UDPController::SetUDPController(UDPController* ptr)
		{
			udpController_ptr = ptr;
		}

		UDPController* MovementActuator2D_UDPController::GetUDPController()
		{
			return udpController_ptr;
		}

		void MovementActuator2D_UDPController::SetCommandDict(SoccerServerCommandDict* dict)
		{
			this->cmd_dict = dict;
		}

		SoccerServerCommandDict* MovementActuator2D_UDPController::GetCommandDict()
		{
			return cmd_dict;
		}

		void MovementActuator2D_UDPController::Dash(double power, double angle)
		{
			args_dash.clear();
			args_dash.emplace_back("" + std::to_string(power)); //team name
			args_dash.emplace_back("" + std::to_string(angle)); //version
			udpController_ptr->SendCommand(cmd_dict->GetCommandTemplate("dash"), cmd_dict->ParseArgsIntoTemplate((cmd_dict->GetArgsTemplate("dash")), args_dash));
		}

		void MovementActuator2D_UDPController::Move(double coords_x, double coords_y)
		{
			args_move.clear();
			args_move.emplace_back("" + std::to_string(coords_x)); //team name
			args_move.emplace_back("" + std::to_string(coords_y)); //version
			udpController_ptr->SendCommand(cmd_dict->GetCommandTemplate("move"), cmd_dict->ParseArgsIntoTemplate((cmd_dict->GetArgsTemplate("move")), args_move));
			std::string init_response = udpController_ptr->GetPacket(); //Send Init command
			std::cout << "Init response: " << init_response << "\n";
		}
	}
}