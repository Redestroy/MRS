#include "BallActuator_UDPController.h"
namespace MRS {
	namespace Device {
		BallActuator_UDPController::BallActuator_UDPController()
		{
			udpController_ptr = nullptr;
		}

		BallActuator_UDPController::BallActuator_UDPController(UDPController * ptr)
		{
			udpController_ptr = ptr;
		}

		BallActuator_UDPController::BallActuator_UDPController(UDPController* ptr, std::string cmd_format) : BallActuator_UDPController(ptr)
		{
			//TODO: replace with ProtocolParser method in the future
			command_template = cmd_format.substr(0, cmd_format.find(';'));
			args_template = cmd_format.substr(cmd_format.find(';') + 1, cmd_format.find('\r') - cmd_format.find(';') - 1);
		}

		BallActuator_UDPController::~BallActuator_UDPController()
		{

		}

		void BallActuator_UDPController::SetUDPController(UDPController* ptr)
		{
			udpController_ptr = ptr;
		}

		UDPController* BallActuator_UDPController::GetUDPController()
		{
			return udpController_ptr;
		}
	}
}