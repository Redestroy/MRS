#pragma once
#include "UDPController.h"
namespace MRS {
	namespace Device {
		class BallActuator_UDPController
		{
		private:
			UDPController* udpController_ptr;
			std::string command_template;
			std::string args_template;
		public:
			BallActuator_UDPController();
			BallActuator_UDPController(UDPController*);
			BallActuator_UDPController(UDPController*, std::string);
			~BallActuator_UDPController();
			void SetUDPController(UDPController*);
			UDPController* GetUDPController();
		};
	}
}

