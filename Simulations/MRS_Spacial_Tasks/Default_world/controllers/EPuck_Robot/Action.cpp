#include "Action.h"



Action::Action()
{
	actionType = TypeDefinitions::NULL_ACTUATOR;
	actionParameter = 0;
}

Action::Action(Action * a)
{
	SetActionType(a->GetActionType());
	SetActionParameter(a->GetActionParameter());
}

Action::Action(TypeDefinitions::ActuatorType type, long long int param)
{
	actionType = type;
	actionParameter = param;
}

Action::Action(std::string * s) : Action()
{
	// instead parse with finding spaces
	SetActionType(TypeDefinitions::CharToAction(s->at(s->find_first_of('_') + 1)));
	std::string temp = "";
	temp.append(s->substr(s->find_first_of(' ') + 1));
	int value = stoi(temp);
	unsigned char firstByte = 0;
	unsigned char secondByte = 0;
	unsigned char thirdByte = 0;
	unsigned char fourthByte = 0;
	if (temp.length() > 2) firstByte = (unsigned char)temp.at(0);
	if (temp.length() > 3) secondByte = (unsigned char)temp.at(1);
	if (temp.length() > 4) thirdByte = (unsigned char)temp.at(2);
	if (temp.length() > 5) fourthByte = (unsigned char)temp.at(3);
	SetActionParameter(value);
	// After remaking the generator
	// SetActionParameter(((long long)fourthByte << 24) | ((long long)thirdByte << 16) | ((long long)secondByte << 8) | ((long long)firstByte));
}

Action::~Action()
{
}

void Action::SetActionType(TypeDefinitions::ActuatorType type)
{
	actionType = type;
}

void Action::SetActionParameter(long long int param)
{
	actionParameter = param;
}

TypeDefinitions::ActuatorType Action::GetActionType()
{
	return actionType;
}

long long int Action::GetActionParameter()
{
	return actionParameter;
}

std::string * Action::toString()
{
	int temp;
	//temp = (int) actionType >> 32;
	data[0] = (char)actionType;
	data[1] = (char)actionType >> 8;
	data[2] = (char)actionType >> 16;
	data[3] = (char)actionType >> 24;
	data[4] = (char)actionParameter;
	data[5] = (char)actionParameter >> 8;
	data[6] = (char)actionParameter >> 16;
	data[7] = (char)actionParameter >> 24;
	data[8] = 'a';//(char)(temp);
	data[9] = 'c'; (char)(temp >> 8);
	data[10] = 't'; (char)(temp >> 16);
	data[11] = 'u'; (char)(temp >> 24);
	data[12] = '\0';
	return new std::string(data);
}
