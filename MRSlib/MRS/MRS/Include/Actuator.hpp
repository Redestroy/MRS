#pragma once
#include "Action.hpp"
#include "Device.hpp"
#include <string>
namespace MRS {
	namespace Device {
		enum class ActuatorType
		{
			DIFFERENTIAL_DRIVE = 'D',
			ACTION_ACTUATOR,
			VIRTUAL_DIGITAL = 'L',
			VIRTUAL_PWM,
			TEXT_OUTPUT,
			NULL_ACTUATOR = 'N',
			DELAY = 'W',
			ACTION_IMPOSSIBLE,
			MOVE_2D
		};

		class Actuator : public Device
		{
		private:
			ActuatorType actuatorType;
			std::string* description;
		protected:
			void SetActuatorType(ActuatorType);

		public:
			Actuator();
			Actuator(std::string*);
			virtual ~Actuator();
			void SetDescription(std::string*);
			std::string* GetDescription();
			virtual void Init();
			virtual void DeInit();
			virtual bool DoAction(Action);
			virtual bool OnDoAction(Action) = 0;
			ActuatorType GetActuatorType();
			virtual std::string* toString() = 0;
		};
	}
}
