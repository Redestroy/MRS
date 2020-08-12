#include "EPuckDrive.h"



EPuckDrive::EPuckDrive()// : DirectDrive()
{
	leftMotor = nullptr;
	rightMotor = nullptr;
	WHEEL_CIRCUMFERENCE = 10.0;
	WHEEL_DISTANCE = 10.0;
	SPEED_PROPORTION = 1;
}

EPuckDrive::EPuckDrive(webots::Robot * robot)
{
	leftMotor = robot->getMotor("left wheel motor");
	rightMotor = robot->getMotor("right wheel motor");
	WHEEL_CIRCUMFERENCE = 10.0;
	WHEEL_DISTANCE = 10.0;
	SPEED_PROPORTION = 1;
	leftMotor->setPosition(INFINITY);
	rightMotor->setPosition(INFINITY);
}

EPuckDrive::EPuckDrive(webots::Motor * left, webots::Motor * right)
{
	leftMotor = left;
	rightMotor = right;
	WHEEL_CIRCUMFERENCE = 128.8;
	WHEEL_DISTANCE = 52.0;
	SPEED_PROPORTION = 1;
	leftMotor->setPosition(INFINITY);
	rightMotor->setPosition(INFINITY);
}


EPuckDrive::~EPuckDrive()
{
	
}

void EPuckDrive::Forward(int speed)
{
	leftMotor->setVelocity(speed*SPEED_PROPORTION);
	rightMotor->setVelocity(speed*SPEED_PROPORTION);
}

void EPuckDrive::Backward(int speed)
{
	leftMotor->setVelocity(-speed*SPEED_PROPORTION);
	rightMotor->setVelocity(-speed*SPEED_PROPORTION);
}

void EPuckDrive::Clockwise(int speed)
{
	leftMotor->setVelocity(speed*SPEED_PROPORTION);
	rightMotor->setVelocity(-speed*SPEED_PROPORTION);
}

void EPuckDrive::CounterClokwise(int speed)
{
	leftMotor->setVelocity(-speed*SPEED_PROPORTION);
	rightMotor->setVelocity(speed*SPEED_PROPORTION);
}

void EPuckDrive::Arc(int speedLeft, int speedRight)
{
	leftMotor->setVelocity(speedLeft*SPEED_PROPORTION);
	rightMotor->setVelocity(speedRight*SPEED_PROPORTION);
}

void EPuckDrive::Raw(long int data)
{
	//later
}

void EPuckDrive::Forward(float speed)
{
	leftMotor->setVelocity(speed*SPEED_PROPORTION);
	rightMotor->setVelocity(speed*SPEED_PROPORTION);
}

void EPuckDrive::Backward(float speed)
{
	leftMotor->setVelocity(-speed * SPEED_PROPORTION);
	rightMotor->setVelocity(-speed * SPEED_PROPORTION);
}

void EPuckDrive::Clockwise(float speed)
{
	leftMotor->setVelocity(speed*SPEED_PROPORTION);
	rightMotor->setVelocity(-speed * SPEED_PROPORTION);
}

void EPuckDrive::CounterClokwise(float speed)
{
	leftMotor->setVelocity(-speed * SPEED_PROPORTION);
	rightMotor->setVelocity(speed*SPEED_PROPORTION);
}

void EPuckDrive::Arc(float speedLeft, float speedRight)
{
	leftMotor->setVelocity(speedLeft*SPEED_PROPORTION);
	rightMotor->setVelocity(speedRight*SPEED_PROPORTION);
}
