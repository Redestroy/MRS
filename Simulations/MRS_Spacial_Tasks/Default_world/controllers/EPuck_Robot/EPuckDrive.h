#pragma once
#include <webots/DifferentialWheels.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>
#include "DirectDrive.h"
class EPuckDrive :
	public DirectDrive
{
	webots::Motor * leftMotor;
	webots::Motor * rightMotor;
	double WHEEL_DISTANCE; // Distance between wheels
	double WHEEL_CIRCUMFERENCE;
	double SPEED_PROPORTION;

public:
	EPuckDrive();
	EPuckDrive(webots::Robot *);
	EPuckDrive(webots::Motor *,webots::Motor *);
	~EPuckDrive();

	void Forward(int);
	void Backward(int);
	void Clockwise(int);
	void CounterClokwise(int);
	void Arc(int, int);
	void Raw(long int);

	void Forward(float);
	void Backward(float);
	void Clockwise(float);
	void CounterClokwise(float);
	void Arc(float, float);
};

