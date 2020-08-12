#pragma once
#include "Condition.h"
#include "ViewPosition2D.h"
#include <iostream>
class ConditionPosition :
	public Condition
{
	float precision;
	float angularPrecision;
	ViewPosition2D * pos;
public:
	ConditionPosition();
	ConditionPosition(ViewPosition2D *, float);
	ConditionPosition(std::string *);
	~ConditionPosition();
	float GetPrecision();
	void SetPrecision(float);
	float GetX();
	void SetX(float);
	float GetY();
	void SetY(float);
	float GetA();
	void SetA(float);
	bool isMet(Worldview *);
	void SetArgsFromString(std::string *);
	std::string * toString();
};

