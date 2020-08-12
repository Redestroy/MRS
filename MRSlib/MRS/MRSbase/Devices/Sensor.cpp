#include "Sensor.hpp"
namespace MRS {
	void Sensor::SetSensorType(SensorType type) {
		this->sensorType = type;
	}

	SensorType Device::GeSensorType() {
		return sensorType;
	}
}