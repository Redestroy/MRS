#pragma once
#include "Device.hpp"
#include <string>
namespace MRS {
	namespace Device {
		class Receiver : public Device
		{
		private:
			//ActuatorType actuatorType;
			//std::string* description;
		protected:
			//void SetActuatorType(ActuatorType);

		public:
			Receiver();
			//Actuator(std::string*);
			~Receiver();
			//void SetDescription(std::string*);
			//std::string* GetDescription();
			virtual void Init();
			virtual void DeInit();
			virtual std::string GetPacket() = 0;

			//void SetEmitter(Emitter*);
			//Emitter* GetEmitter();
			//void SetReceiver(Receiver*);
			//Receiver* GetReceiver();
			//ActuatorType GetActuatorType();
			//virtual std::string* toString() = 0;
		};
	}
}