#pragma once
#include "Action.h"
#include "TypeDefinitions.h"
#include <string>
class Actuator
{
private:
	TypeDefinitions::ActuatorType actuatorType;
	std::string * description;
protected:
	void SetType(TypeDefinitions::ActuatorType);
	
public:
	Actuator();
	Actuator(std::string *);
	~Actuator();
	void SetDescription(std::string *);
	std::string * GetDescription();
	virtual void doAction(Action) = 0;
	TypeDefinitions::ActuatorType GetType();
	virtual std::string * toString() = 0;
};

