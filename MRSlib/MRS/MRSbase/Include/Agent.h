#pragma once
#include <map>
class Agent
{
	int sensor_count;
	int actuator_count;
	std::map<int, Sensor*> sensors;
	std::map<int, Actuator*> actuators;
protected:
	void Init();
	virtual void AddSensor();
	virtual void AddActuator();
public:
	virtual Action NextAction(World*) = 0;
	
};

