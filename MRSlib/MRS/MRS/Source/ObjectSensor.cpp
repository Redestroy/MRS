#include "../Include/ObjectSensor.hpp"

namespace MRS {
	namespace Device {
		ObjectSensor::ObjectSensor()
		{
		}

		ObjectSensor::~ObjectSensor()
		{
		}

		void ObjectSensor::Init()
		{
			Sensor::Init();
		}

		void ObjectSensor::DeInit()
		{
			Sensor::DeInit();
		}

		void ObjectSensor::OnUpdate(long delta_time)
		{
			DetectObject();
		}

	}
}