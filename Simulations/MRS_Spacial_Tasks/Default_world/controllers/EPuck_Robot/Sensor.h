#pragma once
#ifndef SENSOR
#define SENSOR
 // !SENSOR

//#include <string>
#include "TypeDefinitions.h"
#include "Worldview.h"

class Sensor
{
private:
	TypeDefinitions::SensorType sensorType;
	View * sensorView;
	//char sensorInfo [32];
protected:
	void SetType(TypeDefinitions::SensorType);
public:
	Sensor();
	~Sensor();

	TypeDefinitions::SensorType GetType() { 
		return sensorType; 
	};
	void SetView(View *);
	View * GetView();

	virtual void CreateView(Worldview *) = 0;
	virtual void Update() = 0;
	//virtual std::string * toString() = 0;
};

#endif