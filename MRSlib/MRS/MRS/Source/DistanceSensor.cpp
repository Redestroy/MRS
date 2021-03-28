#include "../Include/Sensor.hpp"
#include "../Include/DistanceSensor.hpp"

namespace MRS {
	namespace Device {
		
		DistanceSensor::DistanceSensor()
		{
			distanceView = new DistanceView();
			this->SetSensorView((View *)(distanceView));
			this->SetSensorType(MRS::Device::SensorType::SENSOR_DISTANCE);
		}

		DistanceSensor::~DistanceSensor()
		{
			delete distanceView;
		}

		void DistanceSensor::Init()
		{
			Sensor::Init();
		}

		void DistanceSensor::DeInit()
		{
			Sensor::DeInit();
		}

		void DistanceSensor::OnUpdate(long delta_time)
		{
			distanceView->SetAngle(MeasureAngle());
			distanceView->SetDistance(MeasureDistance());
			distanceView->SetPrecisionAngle(GetAngleError(distanceView->GetAngle(),distanceView->GetDistance()));
			distanceView->SetPrecisionDistance(GetDistanceError(distanceView->GetAngle(), distanceView->GetDistance()));
		}

	}
}