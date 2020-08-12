#pragma once
#ifndef ACTUATORBLOCK
#define ACTUATORBLOCK
// Data structures
#include <stdlib.h>
#include <map>
// Device classes
#include "Actuator.h"
#include "Storage.h" //Class for file management
#include "BaseActuator.h"
class ActuatorBlock
{
private:
#ifdef ARDUINO
	MyMap<type, Actuator> actuators; // Might actually be array, probably will be
#else
	std::multimap<TypeDefinitions::ActuatorType, Actuator *> * actuators;
#endif // ARDUINO
	int actuatorCount;
	bool initialized;
public:
	ActuatorBlock();
	~ActuatorBlock();
	int Init(Storage *, char * filename);
	void Add(Actuator *);
	void Remove(Actuator *);
	void Remove(TypeDefinitions::ActuatorType);
	Actuator * GetActuatorByType(TypeDefinitions::ActuatorType actionType);
};
#endif

