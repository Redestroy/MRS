#pragma once
#include "Device.hpp"
#include "View.hpp"

namespace MRS {
	enum SensorType {
		SENSOR_BASE
	};

	class Sensor : Device
	{
		View* sensorView;
	private:
		SensorType sensorType;
	protected:
		void SetSensorType(SensorType);
	public:
		SensorType GetSensorType();
		virtual View* GetView();
	};
}