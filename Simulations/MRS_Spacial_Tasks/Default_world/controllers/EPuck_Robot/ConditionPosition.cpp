#include "ConditionPosition.h"



ConditionPosition::ConditionPosition() : Condition(TypeDefinitions::CONDITION_POSITION)
{
}

ConditionPosition::ConditionPosition(ViewPosition2D * position, float acc) : ConditionPosition()
{
	pos = position;
	precision = acc;
	angularPrecision = (float)(3.14f * precision / 100.0f);
}

ConditionPosition::ConditionPosition(std::string * conditionString) : ConditionPosition()
{
	this->SetArgsFromString(conditionString);
}


ConditionPosition::~ConditionPosition()
{
}

float ConditionPosition::GetPrecision()
{
	return precision;
}

void ConditionPosition::SetPrecision(float p)
{
	precision = p;
	angularPrecision = (float)(3.14 * precision / 100.0f);
}

float ConditionPosition::GetX()
{
	return pos->GetX();
}

void ConditionPosition::SetX(float x)
{
	pos->SetX(x);
}

float ConditionPosition::GetY()
{
	return pos->GetY();
}

void ConditionPosition::SetY(float y)
{
	pos->SetY(y);
}

float ConditionPosition::GetA()
{
	return pos->GetA();
}

void ConditionPosition::SetA(float a)
{
	pos->SetA(a);
}

bool ConditionPosition::isMet(Worldview * world)
{
	if (world == nullptr) return false;
	//std::cout << "Condition result vs real result  \n";
	//std::cout << "x " << pos->GetX()<<":"<< ((ViewPosition2D *)(world->GetPositionView()))->GetX() <<" \n";
	//std::cout << "y " << pos->GetY() << ":" << ((ViewPosition2D *)(world->GetPositionView()))->GetY() << " \n";
	//std::cout << "a " << pos->GetA() << ":" << ((ViewPosition2D *)(world->GetPositionView()))->GetA() << " \n";
	return (pos->GetX() >= ((ViewPosition2D *)(world->GetPositionView()))->GetX() - precision && pos->GetX() <= ((ViewPosition2D *)(world->GetPositionView()))->GetX() + precision &&
		pos->GetY() >= ((ViewPosition2D *)(world->GetPositionView()))->GetY() - precision && pos->GetY() <= ((ViewPosition2D *)(world->GetPositionView()))->GetY() + precision &&
		pos->GetA() >= ((ViewPosition2D *)(world->GetPositionView()))->GetA() - angularPrecision && pos->GetA() <= ((ViewPosition2D *)(world->GetPositionView()))->GetA() + angularPrecision);
}

void ConditionPosition::SetArgsFromString(std::string * conditionString)
{
	float x = 0, y = 0, a = 0, p = precision;
	sscanf_s(conditionString->c_str(), "/C_S C_P %f %f %f %f", &x, &y, &a, &p); // fix C_S parsing later
	this->pos = new ViewPosition2D(x,y,a);
	this->SetPrecision(p);
}

std::string * ConditionPosition::toString()
{
	char temp[23], temp2[4];
	TypeDefinitions::TypeToString(temp2, TypeDefinitions::Condition, (int)this->GetType());
	sprintf_s(temp, "%s %.2f %.2f %.2f %.2f", temp2, GetX(), GetY(), GetA(), precision);
	return new std::string(temp);
}
