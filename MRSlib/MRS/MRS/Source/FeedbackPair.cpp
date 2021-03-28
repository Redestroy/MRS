#include "../Include/FeedbackPair.hpp"
namespace MRS {
	namespace Device {
		Sensor* FeedbackPair::GetSensor()
		{
			return sensor;
		}
		Actuator* FeedbackPair::GetActuator()
		{
			return actuator;
		}
	}
}