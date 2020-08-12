#include "EpuckLedArray.h"



EpuckLedArray::EpuckLedArray() : DigitalActuatorArray()
{
	for (int i = 0; i < 10; i++) {
		leds[i] = nullptr;
	}
}

EpuckLedArray::EpuckLedArray(webots::Robot * robot) : DigitalActuatorArray()
{
	// is reversed, might need to change later
	leds[outputs[0]] = robot->getLED("led0");
	leds[outputs[1]] = robot->getLED("led1");
	leds[outputs[2]] = robot->getLED("led2");
	leds[outputs[3]] = robot->getLED("led3");
	leds[outputs[4]] = robot->getLED("led4");
	leds[outputs[5]] = robot->getLED("led5");
	leds[outputs[6]] = robot->getLED("led6");
	leds[outputs[7]] = robot->getLED("led7");
	leds[outputs[8]] = robot->getLED("led8");
	leds[outputs[9]] = robot->getLED("led9");
}


EpuckLedArray::~EpuckLedArray()
{
	//delete [] leds;
	//delete leds;
}

void EpuckLedArray::Init()
{
	for (int i = 0; i < ACTUATOR_NUMBER; i++) {
		//set enable
		leds[i]->set(1);
	}
}

void EpuckLedArray::SetOutput(unsigned char pos, bool state)
{
	unsigned char temp = 0;
	temp |= 1 << pos;
	if (pos < ACTUATOR_NUMBER && temp&mask != 0) leds[pos]->set(state);
}

void EpuckLedArray::SetConfiguration(unsigned char config)
{
	unsigned char real = config & mask;
	for (int i = 0; i < ACTUATOR_NUMBER; i++) {
		leds[outputs[i]]->set(0x01 & real);
		real = real >> 1;
	}
}
