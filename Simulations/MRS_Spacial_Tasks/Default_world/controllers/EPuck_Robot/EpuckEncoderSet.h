#pragma once
#include <webots/Robot.hpp>
#include <webots/PositionSensor.hpp>
#include <stdio.h>
#include "Sensor.h"
class EpuckEncoderSet : public Sensor
{
	webots::PositionSensor * leftEncoder;
	webots::PositionSensor * rightEncoder;
	double ENCODER_CPM;
	double countLeft, countRight;		// counts
	double velLeft, velRight;		// counts/s
	int TIME_STEP;
public:
	EpuckEncoderSet();
	EpuckEncoderSet(webots::Robot *);
	EpuckEncoderSet(webots::PositionSensor *, webots::PositionSensor *);
	~EpuckEncoderSet();
	void CreateView(Worldview *);
	void Update();
};

