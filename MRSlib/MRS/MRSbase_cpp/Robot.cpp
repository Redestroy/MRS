#include "Robot.h"


MRSbase::Robot::Robot() : Robot("DEFAULT")
{
}

MRSbase::Robot::Robot(char* name)
{
	name = new char[fmax(std::strlen(name), 1)];
	std::strcpy(name);
}
