#pragma once
#include "Device.hpp"
#include "View.hpp"

namespace MRS {
	namespace Device {
		enum class SensorType {
			SENSOR_BASE,
			SENSOR_DISTANCE,
			SENSOR_DUPLEX
		};

		class Sensor : public Device
		{
		private:	
			View* sensorView;
			SensorType sensorType;
		protected:
			void SetSensorType(SensorType);
			void SetSensorView(View *);
		public:
			SensorType GetSensorType();
			virtual View* GetView();
			virtual void Init();
			virtual void DeInit();
			virtual void Update(long delta_time);
			virtual void OnUpdate(long delta_time) = 0;
		};
	}
}