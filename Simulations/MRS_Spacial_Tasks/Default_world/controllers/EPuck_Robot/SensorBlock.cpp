#include "SensorBlock.h"



SensorBlock::SensorBlock()
{
	sensorCount = 0;
	sensors = new std::multimap<TypeDefinitions::SensorType, Sensor *, std::greater_equal<TypeDefinitions::SensorType>>();
}


SensorBlock::~SensorBlock()
{
	delete[] sensors;
}

void SensorBlock::Init(Storage * local, char * filename)
{

	initialized = true;
}

void SensorBlock::Init(Worldview * world)
{
	CreateViews(world);
	initialized = true;
}

void SensorBlock::CreateViews(Worldview * world) {
	for (std::pair<TypeDefinitions::SensorType, Sensor *> sensor : *sensors) {
		sensor.second->CreateView(world);
	}
}

void SensorBlock::Add(Sensor * nextSensor)
{
	sensors->emplace(nextSensor->GetType(), nextSensor);
	sensorCount++;
}

void SensorBlock::Remove(Sensor *)
{
}

void SensorBlock::Remove(TypeDefinitions::SensorType type)
{
	sensors->erase(type);
	sensorCount--;
}

void SensorBlock::Update(Worldview * world)
{
	for (std::pair<TypeDefinitions::SensorType, Sensor *> sensor : *sensors) {
		sensor.second->Update();
	}
}

