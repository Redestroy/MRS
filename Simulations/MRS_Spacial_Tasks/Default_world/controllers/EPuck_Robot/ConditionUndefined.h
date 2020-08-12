#pragma once
#ifndef CONDITIONUNDEFINED
#define CONDITIONUNDEFINED
#include "Condition.h"
class ConditionUndefined :
	public Condition
{
public:
	ConditionUndefined();
	ConditionUndefined(std::string *);
	~ConditionUndefined();
	bool isMet(Worldview *);
	void SetArgsFromString(std::string *);
	std::string * toString();
};

#endif // !CONDITIONUNDEFINED