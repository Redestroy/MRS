#pragma once
#include "Sensor.hpp"
#include "DistanceView.hpp"
#include <memory>

namespace MRS {
	namespace Device {
		class DistanceSensor : public Sensor {
		private:
			DistanceView* distanceView;
		public:
			DistanceSensor();
			~DistanceSensor();
			virtual void Init();
			virtual void DeInit();
			virtual void OnUpdate(long delta_time);
			virtual double MeasureAngle() = 0;
			virtual double MeasureDistance() = 0;
			virtual double GetAngleError(double angle, double distance) = 0;
			virtual double GetDistanceError(double angle, double distance) = 0;
		};

		template <class T>
		class DummyDistanceSensor : public DistanceSensor {
			typedef DistanceView* (T::* distDetector)(void);
		private:
			//ObjectView
			std::unique_ptr<DistanceView*> object;
			distDetector detector;
			T* overtaker_object;
		public:
			DummyDistanceSensor() : DistanceSensor() {

			};
			~DummyDistanceSensor()
			{
			};
			
			virtual double MeasureAngle() {
				DistanceView* view = (overtaker_object->*detector)();
				return view->GetAngle();
			}

			virtual double MeasureDistance() {
				DistanceView* view = (overtaker_object->*detector)();
				return view->GetDistance();
			};

			virtual double GetAngleError(double, double) {
				DistanceView* view = (overtaker_object->*detector)();
				return view->GetPrecisionAngle();
			};

			virtual double GetDistanceError(double, double) {
				DistanceView* view = new DistanceView();//(overtaker_object->detector)();
				return view->GetPrecisionDistance();
			};

			void SetDetector(distDetector ptr) {
				detector = ptr;
			}
			void SetOvertaker(T* ptr) {
				//overtaker_object = std::make_shared<T*>(ptr);
				overtaker_object = ptr;
			}
		};
	}
}