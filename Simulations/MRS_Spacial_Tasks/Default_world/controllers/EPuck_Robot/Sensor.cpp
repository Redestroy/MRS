#include "Sensor.h"



void Sensor::SetType(TypeDefinitions::SensorType type)
{
	sensorType = type;
}

Sensor::Sensor()
{
}


Sensor::~Sensor()
{
}

void Sensor::SetView(View * view)
{
	sensorView = view;
}

View * Sensor::GetView()
{
	return sensorView;
}
