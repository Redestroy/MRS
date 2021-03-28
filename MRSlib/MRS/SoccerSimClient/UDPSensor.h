#pragma once
#include <iostream>
#include <MRS/Include/Sensor.hpp>
#include "UDPController.h"
#include "UDPPacketReceiver.h"
namespace MRS {
	namespace Device {
		class UDPSensor :
			public Sensor, UDPPacketReceiver
		{
		private:
			UDPController* udpController_ptr;
		public:
			UDPSensor();
			UDPSensor(UDPController*);
			~UDPSensor();
			void SetUDPController(UDPController*);
			UDPController* GetUDPController();
			virtual void OnPacketReceived(std::string);
			virtual void OnUpdate(long);
		};
	}
}

