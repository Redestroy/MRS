#include "SpatialObjectSensor_File.h"

namespace MRS {
	SpatialObjectSensor_File::SpatialObjectSensor_File() : MRS::Device::SpatialObjectSensor()
	{
	}

	SpatialObjectSensor_File::SpatialObjectSensor_File(MRS::Device::ObjectSensor* obj, MRS::Device::DistanceSensor* dist) : MRS::Device::SpatialObjectSensor(obj, dist)
	{
	}

	SpatialObjectSensor_File::~SpatialObjectSensor_File()
	{
	}

	void SpatialObjectSensor_File::OnUpdate(long delta)
	{
		std::string temp = "(see 0.5 (3 4.5 58 5.5 9.3))"; //change to string updater later
		SetData(&temp);
		SpatialObjectSensor::OnUpdate(delta);
	}

	void SpatialObjectSensor_File::SetData(std::string* data)
	{
		this->data.clear();
		this->data = "" + *data;
	}
	std::string* SpatialObjectSensor_File::GetData()
	{
		return &data;
	}
}