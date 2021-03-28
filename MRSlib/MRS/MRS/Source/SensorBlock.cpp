#include "../Include/SensorBlock.hpp"

namespace MRS {
	namespace Device {
		SensorBlock::SensorBlock() : sensors_list()
		{
			sensorCount = 0;
			sensors = new std::multimap<SensorType, Sensor*, std::greater_equal<SensorType>>();
		}


		SensorBlock::~SensorBlock()
		{
			delete[] sensors;
		}

		/* Unused create from file
		void SensorBlock::Init(Storage* local, char* filename)
		{

			initialized = true;
		}
		*/

		void SensorBlock::Init()// Worldview* world)
		{
			//CreateViews(world);
			initialized = true;
			for (std::pair<MRS::Device::SensorType, Sensor*> sensor : *sensors) {
				sensor.second->Init();
			}
		}
		/* OLD
		void SensorBlock::CreateViews(Worldview* world) {
			for (std::pair<TypeDefinitions::SensorType, Sensor*> sensor : *sensors) {
				sensor.second->CreateView(world);
			}
		}
		*/

		void SensorBlock::Add(Sensor* nextSensor)
		{
			sensors->emplace(nextSensor->GetSensorType(), nextSensor);
			sensors_list.emplace_back(nextSensor);
			sensorCount++;
		}

		void SensorBlock::Remove(Sensor*)
		{
		}

		void SensorBlock::Remove(SensorType type)
		{
			sensors->erase(type);
			sensorCount--;
		}

		void SensorBlock::Update(long delta)
		{
			//print sensors
			for (std::pair<MRS::Device::SensorType, Sensor*> sensor : *sensors) {
				sensor.second->Update(delta);
			}
		}

	}
}