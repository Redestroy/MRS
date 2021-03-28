#pragma once
#include <MRS/Include/ObjectSensor.hpp>
#include <string>
namespace MRS {
	class ObjectSensor_Driver :
		public MRS::Device::ObjectSensor
	{
	private:
		std::string data;
	public:
		ObjectSensor_Driver();
		~ObjectSensor_Driver();
		void OnUpdate(long delta_time);
		void DetectObject();
		void SetString(std::string *);
	};
}

