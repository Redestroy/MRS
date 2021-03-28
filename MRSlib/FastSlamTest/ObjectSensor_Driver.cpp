#include "ObjectSensor_Driver.h"

namespace MRS {
	ObjectSensor_Driver::ObjectSensor_Driver()
	{
	}
	ObjectSensor_Driver::~ObjectSensor_Driver()
	{
	}
	void ObjectSensor_Driver::OnUpdate(long delta_time)
	{
		ObjectSensor::OnUpdate(delta_time);
	}

	void ObjectSensor_Driver::DetectObject()
	{
		// parse string for object
	}

	void ObjectSensor_Driver::SetString(std::string* data)
	{
		this->data.clear();
		this->data = "" + *data;
	}
}