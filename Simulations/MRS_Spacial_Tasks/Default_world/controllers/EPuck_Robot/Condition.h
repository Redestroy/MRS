
#ifndef CONDITION
#define CONDITION
#include "TypeDefinitions.h"
#include "Worldview.h"
#include <string>
class Condition
{
	TypeDefinitions::ConditionType conditionType;
protected:
	void SetType(TypeDefinitions::ConditionType);
public:
	Condition();
	Condition(TypeDefinitions::ConditionType);
	~Condition();
	TypeDefinitions::ConditionType GetType() {
		return conditionType;
	};

	virtual bool isMet(Worldview *) = 0;
	virtual std::string * toString() = 0;
};
#endif

