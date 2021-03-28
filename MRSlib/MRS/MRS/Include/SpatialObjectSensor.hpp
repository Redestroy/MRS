#pragma once
#include "DistanceSensor.hpp"
#include "ObjectSensor.hpp"

namespace MRS {
	namespace Device {
		class SpatialObjectSensor : public Sensor {
		private:
			ObjectSensor* objectSensor;
			DistanceSensor* distanceSensor;
		public:
			SpatialObjectSensor();
			SpatialObjectSensor(ObjectSensor *, DistanceSensor *);
			~SpatialObjectSensor();
			virtual void Init();
			virtual void DeInit();
			virtual void OnUpdate(long delta_time);
			void SetObjectSensor(ObjectSensor*);
			void SetDistanceSensor(DistanceSensor*);
		};
	}
}