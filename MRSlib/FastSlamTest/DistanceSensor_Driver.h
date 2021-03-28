#pragma once
#include <MRS/Include/DistanceSensor.hpp>
#include <string>
namespace MRS {
	class DistanceSensor_Driver : 
		public MRS::Device::DistanceSensor
	{
	private:
		std::string data;
		//stringupdater
	public:
		DistanceSensor_Driver();
		~DistanceSensor_Driver();
		virtual void Init();
		virtual void DeInit();
		virtual void OnUpdate(long delta_time);
		virtual double MeasureAngle();
		virtual double MeasureDistance();
		virtual double GetAngleError(double, double);
		virtual double GetDistanceError(double, double);
		void SetSensorData(std::string *);
		double ParseDataForAngle(std::string *);
		double ParseDataForDistance(std::string *);
		//void SetStringUpdater(stringupdater);
	};
}

