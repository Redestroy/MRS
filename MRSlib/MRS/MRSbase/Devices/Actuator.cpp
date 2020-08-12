#include "Actuator.h"



void Actuator::SetType(TypeDefinitions::ActuatorType type)
{
	actuatorType = type;
}

void Actuator::SetDescription(std::string * descrip)
{
	description = descrip;
}

std::string * Actuator::GetDescription()
{
	return description;
}

Actuator::Actuator()
{
}

Actuator::Actuator(std::string * infoString)
{
	//actuatorType = (TypeDefinitions::ActuatorType) ((int) (int)&infoString[0] | (int)&infoString[1]<<8 | (int)&infoString[2] << 16 | (int)&infoString[3] << 24);
}


Actuator::~Actuator()
{
}

TypeDefinitions::ActuatorType Actuator::GetType()
{
	return actuatorType;
}
