#include "DistanceSensor_Driver.h"
#include "MRS/Include/DistanceSensor.hpp"
#define PI 3.14159265

namespace MRS {
	
	DistanceSensor_Driver::DistanceSensor_Driver() : DistanceSensor()
	{
	}

	DistanceSensor_Driver::~DistanceSensor_Driver()
	{
	}

	void DistanceSensor_Driver::Init()
	{
		DistanceSensor::Init();
	}

	void DistanceSensor_Driver::DeInit()
	{
		DistanceSensor::DeInit();
	}

	void DistanceSensor_Driver::OnUpdate(long delta_time)
	{
		//SetData(stringupdater.GetNext());
		DistanceSensor::OnUpdate(delta_time);
	}

	double DistanceSensor_Driver::MeasureAngle()
	{
		return ParseDataForAngle(&data);
	}

	double DistanceSensor_Driver::MeasureDistance()
	{
		return ParseDataForDistance(&data);
	}

	double DistanceSensor_Driver::GetAngleError(double angle, double distance)
	{
		return 0.2;
	}

	double DistanceSensor_Driver::GetDistanceError(double angle, double distance)
	{
		return 0.01;
	}

	void DistanceSensor_Driver::SetSensorData(std::string* data)
	{
		this->data.clear();
		this->data = this->data + *data;
	}

	double DistanceSensor_Driver::ParseDataForAngle(std::string* data)
	{
		double angle = 0.0;
		int currentParentesiesOpen = 0, next = 0, firstTemp, secondTemp;
		std::string temp, dataSubstring, angleSubstring;
		// copy
		temp = "" + *data;
		// get substring
		currentParentesiesOpen = (int)temp.find_first_of('(', 0);
		next = (int)temp.find_first_of('(', currentParentesiesOpen + 1);
		currentParentesiesOpen = next;
		next = (int)temp.find_first_of('(', currentParentesiesOpen + 1);
		dataSubstring = "" + temp.substr(currentParentesiesOpen + 1, next - currentParentesiesOpen);
		firstTemp = (int)dataSubstring.find_first_of(' ', 0);
		secondTemp = (int)dataSubstring.find_first_of(' ', firstTemp + 1);
		firstTemp = secondTemp;
		secondTemp = (int)dataSubstring.find_first_of(' ', firstTemp + 1);
		angleSubstring = "" + dataSubstring.substr(firstTemp + 1, secondTemp - firstTemp);
		// parse double
		try {
			angle = std::stod(angleSubstring);
		}
		catch (...) {
			angle = -4.43;
			printf(" Sub: %s \n", angleSubstring.c_str());
			printf(" Num error \n");
		}
		// normalize
		angle = angle*PI/180.0;
		return angle;
	}

	double DistanceSensor_Driver::ParseDataForDistance(std::string* data)
	{
		double distance = 0.0;
		int currentParentesiesOpen = 0, next = 0, firstTemp, secondTemp;
		std::string temp, dataSubstring, distanceSubstring;
		// copy
		temp = "" + *data;
		// get substring
		currentParentesiesOpen = (int)temp.find_first_of('(', 0);
		next = (int)temp.find_first_of('(', currentParentesiesOpen + 1);
		currentParentesiesOpen = next;
		next = (int)temp.find_first_of('(', currentParentesiesOpen + 1);
		dataSubstring = "" + temp.substr(currentParentesiesOpen + 1, next - currentParentesiesOpen);
		firstTemp = (int)dataSubstring.find_first_of(' ', 0);
		secondTemp = (int)dataSubstring.find_first_of(' ', firstTemp + 1);
		distanceSubstring = "" + dataSubstring.substr(firstTemp + 1, secondTemp - firstTemp);
		// parse double
		try {
			distance = std::stod(distanceSubstring);
		}
		catch (...) {
			distance = -4.43;
			printf(" Num error \n");
		}
		// normalize
		return distance;
	}

}