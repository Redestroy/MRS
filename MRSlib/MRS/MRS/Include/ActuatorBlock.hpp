#pragma once
#include "Actuator.hpp"
#include "View.hpp"
#include <deque>
#include <map>

namespace MRS {
	namespace Device {
		class ActuatorBlock
		{
		private:
			std::multimap<MRS::Device::ActuatorType, Actuator*>* actuators;
			std::deque<Actuator*> actuators_list;
			int actuatorCount;
			bool initialized;
		public:
			ActuatorBlock();
			~ActuatorBlock();
			//int Init(Storage*, char* filename);
			int Init();
			bool DoAction(Action *);
			void Add(Actuator*);
			void Remove(Actuator*);
			void Remove(MRS::Device::ActuatorType);
			Actuator* GetActuatorByType(MRS::Device::ActuatorType actionType);
		};
	}
}