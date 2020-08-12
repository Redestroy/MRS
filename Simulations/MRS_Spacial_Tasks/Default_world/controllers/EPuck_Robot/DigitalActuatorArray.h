#pragma once
#ifndef DIGITALACTUATORARRAY
#define DIGITALACTUATORARRAY
#include "Actuator.h"

class DigitalActuatorArray :
	public Actuator
{
private:
	unsigned char configuration;
	unsigned char defaultStates;
protected:
	unsigned char mask;
	unsigned static char ACTUATOR_NUMBER;
	int * outputs;
	int actuatorCount;
public:
	DigitalActuatorArray();
	DigitalActuatorArray(unsigned char);
	~DigitalActuatorArray();
	void AddActuator(unsigned char, int);
	void AddActuator(unsigned char, int, bool);
	void doAction(Action);
	virtual void SetOutput(unsigned char, bool) = 0;
	virtual void SetConfiguration(unsigned char) = 0;
	std::string * toString();
};

#endif // !1