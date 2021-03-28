#include "../Include/SpatialObjectSensor.hpp"

namespace MRS {
	namespace Device {
		
		SpatialObjectSensor::SpatialObjectSensor()
		{
			SetSensorType(MRS::Device::SensorType::SENSOR_DUPLEX);
			objectSensor = new DummyObjectSensor<SpatialObjectSensor>();
			distanceSensor = new DummyDistanceSensor<SpatialObjectSensor>();
			//set detectors if needed
		}

		SpatialObjectSensor::SpatialObjectSensor(ObjectSensor* objSensor, DistanceSensor* distSensor) : SpatialObjectSensor()
		{
			objectSensor = objSensor;
			distanceSensor = distSensor;
		}

		SpatialObjectSensor::~SpatialObjectSensor()
		{
			if(((MRS::Device::Device*)this)->IsInitialized())DeInit();
		}

		void SpatialObjectSensor::Init()
		{
			if (distanceSensor != nullptr && objectSensor != nullptr) {
				distanceSensor->Init();
				objectSensor->Init();
				Sensor::Init();
			}
		}

		void SpatialObjectSensor::DeInit()
		{
			if (distanceSensor != nullptr) distanceSensor->DeInit();
			if (objectSensor != nullptr) objectSensor->DeInit();
			Sensor::DeInit();
		}
		
		void SpatialObjectSensor::OnUpdate(long delta_time)
		{
			distanceSensor->OnUpdate(delta_time);
			objectSensor->OnUpdate(delta_time);
		}

		void SpatialObjectSensor::SetObjectSensor(ObjectSensor* sensor)
		{
			objectSensor = sensor;
		}

		void SpatialObjectSensor::SetDistanceSensor(DistanceSensor* sensor)
		{
			distanceSensor = sensor;
		}

	}
}