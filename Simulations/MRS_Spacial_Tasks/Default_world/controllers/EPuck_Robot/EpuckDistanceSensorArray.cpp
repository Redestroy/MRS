#include "EpuckDistanceSensorArray.h"



EpuckDistanceSensorArray::EpuckDistanceSensorArray() : AnalogSensor()
{
	for (int i = 0; i < 8; i++) {
		distanceSensors[i] = nullptr;
	}
}

EpuckDistanceSensorArray::EpuckDistanceSensorArray(webots::Robot * robot) : AnalogSensor()
{
	//DISTANCE_SENSOR_AMOUNT = 8;
	distanceSensors[0] = robot->getDistanceSensor("ps0");
	distanceSensors[1] = robot->getDistanceSensor("ps1");
	distanceSensors[2] = robot->getDistanceSensor("ps2");
	distanceSensors[3] = robot->getDistanceSensor("ps3");
	distanceSensors[4] = robot->getDistanceSensor("ps4");
	distanceSensors[5] = robot->getDistanceSensor("ps5");
	distanceSensors[6] = robot->getDistanceSensor("ps6");
	distanceSensors[7] = robot->getDistanceSensor("ps7");
	TIME_STEP = 8;
	for (int i = 0; i < 8; i++) {
		distanceSensors[i]->enable(TIME_STEP);
	}
}


EpuckDistanceSensorArray::~EpuckDistanceSensorArray()
{
}

double EpuckDistanceSensorArray::Read(unsigned char pos)
{
	//printf("val: %f",distanceSensors[pos]->getValue());
	return distanceSensors[pos]->getValue();
}
