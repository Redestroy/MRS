#pragma once
#ifndef SENSORBLOCK
#define SENSORBLOCK
#include <map>
#include "TypeDefinitions.h"
#include "Sensor.h"
#include "Storage.h"
#include "Worldview.h"
class SensorBlock
{
private:
	std::multimap<TypeDefinitions::SensorType, Sensor *, std::greater_equal<TypeDefinitions::SensorType>> * sensors;
	int sensorCount;
	bool initialized;
public:
	SensorBlock();
	~SensorBlock();
	void Init(Storage *, char * filename);
	void Init(Worldview * world);
	void CreateViews(Worldview * world);
	void Add(Sensor *);
	void Remove(Sensor *);
	void Remove(TypeDefinitions::SensorType);
	void Update(Worldview *);
};
#endif
