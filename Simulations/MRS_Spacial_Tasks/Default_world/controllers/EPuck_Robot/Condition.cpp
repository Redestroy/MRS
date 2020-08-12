#include "Condition.h"



void Condition::SetType(TypeDefinitions::ConditionType type)
{
	conditionType = type;
}

Condition::Condition() : Condition(TypeDefinitions::CONDITION_UNDEFINED)
{
}

Condition::Condition(TypeDefinitions::ConditionType type)
{
	conditionType = type;
}


Condition::~Condition()
{
}
