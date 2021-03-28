#include "BodyInformationSensor_UDPController.h"
namespace MRS {
	namespace Device {
		BodyInformationSensor_UDPController::BodyInformationSensor_UDPController()
		{
			udpController_ptr = nullptr;
		}

		BodyInformationSensor_UDPController::BodyInformationSensor_UDPController(UDPController* ptr)
		{
			udpController_ptr = ptr;
		}

		BodyInformationSensor_UDPController::BodyInformationSensor_UDPController(UDPController* ptr, std::string cmd_format) : BodyInformationSensor_UDPController(ptr)
		{
				//TODO: replace with ProtocolParser method in the future
				command_template = cmd_format.substr(0, cmd_format.find(';'));
				args_template = cmd_format.substr(cmd_format.find(';') + 1, cmd_format.find('\r') - cmd_format.find(';') - 1);
		}

		BodyInformationSensor_UDPController::~BodyInformationSensor_UDPController()
		{
		}

		void BodyInformationSensor_UDPController::SetUDPController(UDPController* ptr)
		{
			this->udpController_ptr = ptr;
		}

		UDPController* BodyInformationSensor_UDPController::GetUDPController()
		{
			return udpController_ptr;
		}
	}
}