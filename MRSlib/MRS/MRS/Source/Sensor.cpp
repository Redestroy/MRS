//#include "pch.h"
//#include "Sensor.hpp"
#include "../Include/Sensor.hpp"

namespace MRS {
	namespace Device {
		void Sensor::SetSensorType(SensorType type) {
			this->sensorType = type;
		}

		void Sensor::SetSensorView(View* view)
		{
			this->sensorView = view;
		}

		SensorType Sensor::GetSensorType() {
			return sensorType;
		}

		View* Sensor::GetView()
		{
			return sensorView;
		}

		void Sensor::Init()
		{
			Device::Init();
		}
		void Sensor::DeInit()
		{
			Device::DeInit();
		}

		void Sensor::Update(long delta_time)
		{
			if (this->IsInitialized()) {
				OnUpdate(delta_time);
			}
		}
	}
}