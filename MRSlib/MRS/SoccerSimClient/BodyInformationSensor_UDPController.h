#pragma once
#include "UDPController.h"
namespace MRS {
	namespace Device {
		class BodyInformationSensor_UDPController //: InformationSensor
		{
		private:
			UDPController* udpController_ptr;
			std::string command_template;
			std::string args_template;
			//PlayerBodyInformationView* information;
		public:
			BodyInformationSensor_UDPController();
			BodyInformationSensor_UDPController(UDPController*);
			BodyInformationSensor_UDPController(UDPController*, std::string);
			~BodyInformationSensor_UDPController();
			void SetUDPController(UDPController*);
			UDPController* GetUDPController();
			//PlayerBodyInformationView* GetBodyInformation();
			//void SetBodyInformation(PlayerBodyInformationView*);
		};
	}
}

