#pragma once
#include "Sensor.h"
#include "DistanceView.h"

class AnalogSensor :
	public Sensor
{
protected:
	int input1, input2, input3;
	DistanceView * distances;
public:
	AnalogSensor();
	AnalogSensor(int);
	void CreateView(Worldview *);
	void Update();
	virtual double Read(unsigned char) = 0;
	~AnalogSensor();
};