#include "../Include/Agent.hpp"
namespace MRS {
	namespace Task {
		void Agent::SetSensors(SensorBlock* sensors)
		{
			this->sensors = sensors;
		}
		void Agent::SetActuators(ActuatorBlock* actuators)
		{
			this->actuators = actuators;
		}
		Device::SensorBlock* Agent::GetSensors()
		{
			return sensors;
		}
		Device::ActuatorBlock* Agent::GetActuators()
		{
			return actuators;
		}
	}
}


