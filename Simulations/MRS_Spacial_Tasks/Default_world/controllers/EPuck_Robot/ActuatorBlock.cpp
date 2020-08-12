#include "ActuatorBlock.h"


ActuatorBlock::ActuatorBlock()
{
	actuatorCount = 0;
	actuators = new std::multimap<TypeDefinitions::ActuatorType, Actuator *>;
}

ActuatorBlock::~ActuatorBlock()
{
	delete [] actuators;
}

int ActuatorBlock::Init(Storage * localStorage, char * filename) 
{
	std::cout << "initializing";
	return 1;
	/*
	FILE * file = fopen(filename, "r");//localStorage.OpenFile(filename);
	char buffer [1024];
	char * infoString;
	char * temp;
	try {
		if (file == NULL) return -1;
		while (!feof(file)) {
			if (fgets(buffer, 1024, file) == NULL) break;
		}
		infoString = strtok_s(buffer ,"\n", &temp);
		while (infoString != NULL) {
			//strncpy(infoString, buffer, a-buffer+1);
			this->Add(new BaseActuator(&std::string(infoString)));
			infoString = strtok_s(NULL, "\n", &temp);
		}
		fclose(file);
		initialized = true;
	}
	catch (int e) {
		return e;
	}*/
}

void ActuatorBlock::Add(Actuator * nextActuator)
{
	// Add some safety, check insert
	actuators->emplace( nextActuator->GetType(), nextActuator );
	actuatorCount++;
}

void ActuatorBlock::Remove(Actuator * prevActuator)
{
	//actuators->erase();
	delete prevActuator;
}

void ActuatorBlock::Remove(TypeDefinitions::ActuatorType type)
{
	// Add some safety
	actuators->erase(type);
	actuatorCount--;
}

Actuator * ActuatorBlock::GetActuatorByType(TypeDefinitions::ActuatorType actionType) {
	//Add dictionary to convert specific action types to actuator types

	switch (actionType) {
		case 1:
		break;
		default:
			if ((actuators->count(actionType) != (size_t) 0) ) {
				return actuators->find(actionType)->second;
			}
			else return nullptr;
		break;
	}
	return nullptr;
}
