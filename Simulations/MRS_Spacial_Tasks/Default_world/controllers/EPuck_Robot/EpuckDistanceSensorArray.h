#pragma once
#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include "AnalogSensor.h"
class EpuckDistanceSensorArray : public AnalogSensor
{
	webots::DistanceSensor * distanceSensors[8];
	int TIME_STEP;
	//int DISTANCE_SENSOR_AMOUNT;
public:
	EpuckDistanceSensorArray();
	EpuckDistanceSensorArray(webots::Robot *);
	~EpuckDistanceSensorArray();
	double Read(unsigned char);
};

