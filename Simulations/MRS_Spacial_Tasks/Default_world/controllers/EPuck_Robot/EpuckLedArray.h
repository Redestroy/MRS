#pragma once
#include <webots/LED.hpp>
#include <webots/Robot.hpp>
#include "DigitalActuatorArray.h"
class EpuckLedArray : public DigitalActuatorArray
{
	webots::LED * leds[10];

public:
	EpuckLedArray();
	EpuckLedArray(webots::Robot *);
	~EpuckLedArray();
	void Init();
	void SetOutput(unsigned char, bool);
	void SetConfiguration(unsigned char);

};

