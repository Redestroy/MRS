#include "DirectDrive.h"


DirectDrive::DirectDrive() : Actuator()
{
	this->SetType(TypeDefinitions::DIFFERENTIAL_DRIVE);
}


DirectDrive::~DirectDrive()
{
}

void DirectDrive::doAction(Action action)
{
	switch (action.GetActionType()) {
		case TypeDefinitions::FORWARD:
			Forward((int)action.GetActionParameter());
			break;
		case TypeDefinitions::BACKWARD:
			Backward((int)action.GetActionParameter());
			break;
		case TypeDefinitions::CLOCKWISE:
			Clockwise((int)action.GetActionParameter());
			break;
		case TypeDefinitions::COUNTER_CLOCKWISE:
			CounterClokwise((int)action.GetActionParameter());
			break;
		case TypeDefinitions::ARC_DRIVE:
			CounterClokwise((int)action.GetActionParameter());
			break;
		case TypeDefinitions::RAW_DRIVE:
			CounterClokwise((int)action.GetActionParameter());
			break;
	}
}

std::string * DirectDrive::toString()
{
	SetDescription(&std::string("string"));
	return this->GetDescription();
}
