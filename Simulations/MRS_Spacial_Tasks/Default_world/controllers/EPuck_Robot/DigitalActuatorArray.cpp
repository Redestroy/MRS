#include "DigitalActuatorArray.h"

unsigned char DigitalActuatorArray::ACTUATOR_NUMBER = 10;

DigitalActuatorArray::DigitalActuatorArray() : DigitalActuatorArray((unsigned char)0xFF)
{
}

DigitalActuatorArray::DigitalActuatorArray(unsigned char mask) : Actuator()
{
	outputs = new int[ACTUATOR_NUMBER];
	this->mask = mask;
	configuration = 0;
	defaultStates = 0xFF;
	actuatorCount = 0;
	SetType(TypeDefinitions::VIRTUAL_DIGITAL);
	for (int i = 0; i < ACTUATOR_NUMBER; i++) outputs[i] = i;
}


DigitalActuatorArray::~DigitalActuatorArray()
{
	delete[] outputs;
}

void DigitalActuatorArray::AddActuator(unsigned char pos, int pin) 
{
	AddActuator(pos, pin, true);
}

void DigitalActuatorArray::AddActuator(unsigned char pos, int pin, bool normallyLow)
{
	if (pos < ACTUATOR_NUMBER) {
		outputs[pos] = pin;
		defaultStates |= (normallyLow << pos);
		actuatorCount++;
	}
}

void DigitalActuatorArray::doAction(Action action)
{
	switch (action.GetActionType()) {
	case TypeDefinitions::VIRTUAL_DIGITAL:
		this->SetConfiguration((unsigned char)action.GetActionParameter());
		break;
	default:
		return;
	}
}

std::string * DigitalActuatorArray::toString()
{
	SetDescription(&std::string("led"));
	return GetDescription();
}
