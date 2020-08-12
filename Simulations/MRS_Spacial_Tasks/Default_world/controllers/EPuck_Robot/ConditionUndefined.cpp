#include "ConditionUndefined.h"



ConditionUndefined::ConditionUndefined()
{
	SetType(TypeDefinitions::CONDITION_UNDEFINED);
}

ConditionUndefined::ConditionUndefined(std::string *) : ConditionUndefined()
{

}


ConditionUndefined::~ConditionUndefined()
{
}



bool ConditionUndefined::isMet(Worldview * world)
{
	return false;
}

void ConditionUndefined::SetArgsFromString(std::string *)
{

}

std::string * ConditionUndefined::toString()
{
	char temp[4]; //temp2[4];
	TypeDefinitions::TypeToString(temp, TypeDefinitions::Condition, (int)this->GetType());
	//sprintf(temp, "%s %.2f %.2f %.2f %.2f", temp2);
	return new std::string(temp);
}
