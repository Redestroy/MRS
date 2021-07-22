#pragma once
#include "Sensor.hpp"
#include "InformationView.hpp"
#include <memory>
namespace MRS {
	namespace Device {
		class InformationSensor : public Sensor {
		private:
			//ObjectView
			std::unique_ptr<InformationView*> information;
			bool update_all;
		public:
			InformationSensor();
			~InformationSensor();
			virtual void Init();
			virtual void DeInit();
			virtual void OnUpdate(long delta_time);
			void SetUpdateAll();
			virtual std::string UpdateParameter(std::string identifier) = 0;
		};


		template <class T>
		class DummyInformationSensor : public InformationSensor {
			//typedef std::unique_ptr<ObjectView*> (T::* objDetector)(void);
			typedef std::string (T::* infoDetector)(std::string);
		private:
			//ObjectView
			std::unique_ptr<InformationView> object;
			infoDetector detector;
			T* overtaker_object;
		public:
			DummyInformationSensor() : InformationSensor() {

			};

			~DummyInformationSensor()
			{
			};

			virtual void UpdateParameter(std::string identifier) {
				//fill object view with something
				 std::string i = (overtaker_object->*detector)(identifier);
				//DistanceView* view = (overtaker_object->*detector)();
			};

			void SetDetector(infoDetector* ptr) {
				detector = ptr;
			}
			void SetOvertaker(T* ptr) {
				overtaker_object = std::make_shared<T*>(ptr);
			}

		};
	}
}