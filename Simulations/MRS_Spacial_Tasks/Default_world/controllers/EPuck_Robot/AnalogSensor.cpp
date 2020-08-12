#include "AnalogSensor.h"



AnalogSensor::AnalogSensor():AnalogSensor(0)
{

}

AnalogSensor::AnalogSensor(int pin)
{
	input1 = pin;
	SetType(TypeDefinitions::SENSOR_GRADIENT);
}

void AnalogSensor::CreateView(Worldview * world)
{
	distances = new DistanceView();
	View * gradient = distances;
	SetView(gradient);
	Update();
	world->SetFloorView(gradient);
}

void AnalogSensor::Update()
{
	for (unsigned char i = 0; i < 8; i++) {
		((DistanceView *)(this->GetView()))->SetDistance(i,Read(i));
		//this->GetView()->SetData(i * 2, (unsigned char)data[i]);
		//this->GetView()->SetData(i * 2 + 1, (unsigned char)((data[i] >> 8)&0b00000011));
	}
}

AnalogSensor::~AnalogSensor()
{
}
