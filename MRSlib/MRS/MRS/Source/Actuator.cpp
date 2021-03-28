//#include "pch.h"
#include "Actuator.hpp"

namespace MRS {
	namespace Device {
		void Actuator::SetActuatorType(ActuatorType type)
		{
			actuatorType = type;
		}

		void Actuator::SetDescription(std::string* descrip)
		{
			description = descrip;
		}

		std::string* Actuator::GetDescription()
		{
			return description;
		}

		void Actuator::Init()
		{
			Device::Init();
		}

		void Actuator::DeInit()
		{
			Device::DeInit();
		}

		bool Actuator::DoAction(Action a)
		{
			if (IsInitialized()) {
				return OnDoAction(a);
			}
			return false;
		}

		Actuator::Actuator()
		{
			Device::SetDeviceType(MRS::Device::DeviceType::ACTUATOR);
			actuatorType = ActuatorType::NULL_ACTUATOR;
		}

		Actuator::Actuator(std::string* infoString)
		{
			//TODO: parse string to initialize actuato
			Device::SetDeviceType(MRS::Device::DeviceType::ACTUATOR);
			actuatorType = ActuatorType::NULL_ACTUATOR;
			//actuatorType = (TypeDefinitions::ActuatorType) ((int) (int)&infoString[0] | (int)&infoString[1]<<8 | (int)&infoString[2] << 16 | (int)&infoString[3] << 24);
		}


		Actuator::~Actuator()
		{
		}

		ActuatorType Actuator::GetActuatorType()
		{
			return actuatorType;
		}
	}
}