#pragma once
#include "UDPController.h"
namespace MRS {
	namespace Device {
		class AudialSensor_UDPController
		{
		private:
			UDPController* udpController_ptr;
			std::string command_template;
			std::string args_template;

			std::string response_format_template;  // or replaced by an abstract function
		public:
			AudialSensor_UDPController();
			AudialSensor_UDPController(UDPController*);
			AudialSensor_UDPController(UDPController*, std::string);
			~AudialSensor_UDPController();
			void SetUDPController(UDPController*);
			UDPController* GetUDPController();
		};
	}
}

