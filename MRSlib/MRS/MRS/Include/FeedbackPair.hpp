#pragma once
#include "Sensor.hpp"
#include "Actuator.hpp"
#include <deque>			//might need to implament seperatly
namespace MRS {
	namespace Device {
		class
#ifdef BUILD_DLL
			__declspec(dllexport)
#endif // BUILD_DLL
			 FeedbackPair : public Device 
		{
		private:
			Actuator* actuator;
			Sensor* sensor;
		public:
			Sensor* GetSensor();
			Actuator* GetActuator();
			virtual float ErrorValue() = 0; //Difference between set value and real value 

		};
	}
}