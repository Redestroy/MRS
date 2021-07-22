#pragma once
#include "Device.hpp"
#include "SensorBlock.hpp"
#include "ActuatorBlock.hpp"
#include <deque>			//might need to implament seperatly
namespace MRS {
	namespace Task {
		class 
#ifdef BUILD_DLL
			__declspec(dllexport)
#endif // BUILD_DLL
			Agent {
		private:
			Device::SensorBlock * sensors;
			Device::ActuatorBlock* actuators;
		public:
			void SetSensors(Device::SensorBlock*);
			void SetActuators(Device::ActuatorBlock*);
			Device::SensorBlock * GetSensors();
			Device::ActuatorBlock* GetActuators();

			virtual void Update() = 0;
			virtual void DoAction(Device::Action) = 0;
		};
	}
}