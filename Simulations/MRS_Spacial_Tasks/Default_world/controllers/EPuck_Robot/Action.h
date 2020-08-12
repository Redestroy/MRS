#pragma once
#ifndef ACTION
#define ACTION
#include "TypeDefinitions.h"
#include <string>
class Action
{
private:
	TypeDefinitions::ActuatorType actionType;
	long long int actionParameter;
	char data[13];
public:
	Action();
	Action(Action *);
	Action(TypeDefinitions::ActuatorType, long long int);
	Action(std::string *);
	~Action();

	void SetActionType(TypeDefinitions::ActuatorType);
	void SetActionParameter(long long int);

	TypeDefinitions::ActuatorType GetActionType();
	long long int GetActionParameter();

	std::string * toString();
};
#endif
