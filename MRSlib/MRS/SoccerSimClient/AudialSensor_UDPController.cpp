#include "AudialSensor_UDPController.h"
namespace MRS {
	namespace Device {
		AudialSensor_UDPController::AudialSensor_UDPController()
		{
			udpController_ptr = nullptr;
		}

		AudialSensor_UDPController::AudialSensor_UDPController(UDPController* ptr)
		{
			udpController_ptr = ptr;
		}

		AudialSensor_UDPController::AudialSensor_UDPController(UDPController* ptr, std::string cmd_format) : AudialSensor_UDPController( ptr)
		{
			//TODO: replace with ProtocolParser method in the future
			command_template = cmd_format.substr(0, cmd_format.find(';'));
			args_template = cmd_format.substr(cmd_format.find(';') + 1, cmd_format.find('\r') - cmd_format.find(';') - 1);
		}

		AudialSensor_UDPController::~AudialSensor_UDPController()
		{
		}

		void AudialSensor_UDPController::SetUDPController(UDPController* ptr)
		{
			udpController_ptr = ptr;
		}

		UDPController* AudialSensor_UDPController::GetUDPController()
		{
			return udpController_ptr;
		}
	}
}