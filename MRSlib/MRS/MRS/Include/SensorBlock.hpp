#pragma once
#include "Sensor.hpp"
#include "View.hpp"
#include <deque>
#include <map>

namespace MRS {
	namespace Device {
		class SensorBlock
		{
		private:
			std::deque<Sensor*> sensors_list;
			std::deque<View*> views;	//TODO: make read only
			std::multimap<MRS::Device::SensorType, Sensor*, std::greater_equal<MRS::Device::SensorType>>* sensors;
			int sensorCount;
			bool initialized;
		public:
			SensorBlock();
			~SensorBlock();
			//void Init(Storage*, char* filename);
			void Init();//Worldview* world);
			//void CreateViews(Worldview* world); Depricated (maybe)
			void Add(Sensor*);
			void Remove(Sensor*);
			void Remove(MRS::Device::SensorType);
			void Update(long delta);
		};
	}
}