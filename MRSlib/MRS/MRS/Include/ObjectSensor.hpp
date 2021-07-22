#pragma once
#include "Sensor.hpp"
#include "ObjectView.hpp"
#include <memory>
namespace MRS {
	namespace Device {
		class ObjectSensor : public Sensor{
		private:
			//ObjectView
			std::unique_ptr<ObjectView*> object;
		public:
			ObjectSensor();
			~ObjectSensor();
			virtual void Init();
			virtual void DeInit();
			virtual void OnUpdate(long delta_time);
			virtual void DetectObject() = 0;
		};


		template <class T>
		class DummyObjectSensor : public ObjectSensor {
			//typedef std::unique_ptr<ObjectView*> (T::* objDetector)(void);
			typedef int (T::* objDetector)(void);
		private:
			//ObjectView
			std::unique_ptr<ObjectView> object;
			objDetector detector;
			T* overtaker_object;
		public:
			DummyObjectSensor() : ObjectSensor() {
			
			};
			~DummyObjectSensor()
			{
			};
			virtual void DetectObject() {
				//fill object view with something
				int i = (overtaker_object->*detector)();
				//DistanceView* view = (overtaker_object->*detector)();
			};

			void SetDetector(objDetector ptr) {
				detector = ptr;
			}
			void SetOvertaker(T* ptr) {
				//overtaker_object = std::make_shared<T*>(ptr);
				overtaker_object = ptr;
			}
		};
	}
}