#pragma once
#ifndef CONDITIONNULL
#define CONDITIONNULL
#include "Condition.h"
#include <string>
class ConditionNull :
	public Condition
{
public:
	ConditionNull();
	ConditionNull(std::string *);
	~ConditionNull();
	bool isMet(Worldview *);
	std::string * toString();
};
#endif // !CONDITIONNULL