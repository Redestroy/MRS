#pragma once
#ifndef DIRECTDRIVE
#define DIRECTDRIVE
#include "Actuator.h"
#include "TypeDefinitions.h"
class DirectDrive :
	public Actuator
{
private:
	int leftDirPin1;
	int leftDirPin2;
	int leftSpeedPin;
	int rightDirPin1;
	int rightDirPin2;
	int rightSpeedPin;
public:
	DirectDrive();
	~DirectDrive();
	void doAction(Action);
	virtual void Forward(int) = 0;
	virtual void Backward(int) = 0;
	virtual void Clockwise(int) = 0;
	virtual void CounterClokwise(int) = 0;
	virtual void Arc(int, int) = 0;
	virtual void Raw(long int) = 0;
	virtual void Forward(float) = 0;
	virtual void Backward(float) = 0;
	virtual void Clockwise(float) = 0;
	virtual void CounterClokwise(float) = 0;
	virtual void Arc(float, float) = 0;
	//virtual void Raw(long int) = 0;
	std::string * toString();
};
#endif
