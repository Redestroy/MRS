#pragma once
#include "Device.hpp"
#include "Emitter.hpp"
#include "Receiver.hpp"
#include <string>
namespace MRS {
	namespace Device {
		class CommunicationDevice : public Device
		{
		private:
			//ActuatorType actuatorType;
			//std::string* description;
			Emitter* emitter;
			Receiver* receiver;
		protected:
			//void SetActuatorType(ActuatorType);

		public:
			CommunicationDevice();
			//Actuator(std::string*);
			~CommunicationDevice();
			//void SetDescription(std::string*);
			//std::string* GetDescription();
			virtual void Init();
			virtual void DeInit();
			virtual void SendMessage(std::string) = 0;
			virtual std::string GetPacket() = 0;

			void SetEmitter(Emitter*);
			Emitter* GetEmitter();
			void SetReceiver(Receiver*);
			Receiver* GetReceiver();
			//ActuatorType GetActuatorType();
			//virtual std::string* toString() = 0;
		};
	}
}