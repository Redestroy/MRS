#pragma once
#include <MRS/Include/SpatialObjectSensor.hpp>
#include <string>
namespace MRS {
	class SpatialObjectSensor_File : public MRS::Device::SpatialObjectSensor
	{
	private:
		std::string data;
		//string updater
	public:
		SpatialObjectSensor_File();
		SpatialObjectSensor_File(MRS::Device::ObjectSensor*, MRS::Device::DistanceSensor*);
		~SpatialObjectSensor_File();
		virtual void OnUpdate(long delta);
		void SetData(std::string *);
		std::string * GetData();

	};
}
