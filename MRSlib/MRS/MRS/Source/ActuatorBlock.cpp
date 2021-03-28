#include "../Include/ActuatorBlock.hpp"

namespace MRS {
	namespace Device {
		ActuatorBlock::ActuatorBlock()
		{
			actuatorCount = 0;
			actuators = new std::multimap<MRS::Device::ActuatorType, Actuator*>;
		}

		ActuatorBlock::~ActuatorBlock()
		{
			delete[] actuators;
		}

		/* unused read actuator info from file
		int ActuatorBlock::Init(Storage* localStorage, char* filename)
		{
			std::cout << "initializing";
			return 1;
			
			FILE * file = fopen(filename, "r");//localStorage.OpenFile(filename);
			char buffer [1024];
			char * infoString;
			char * temp;
			try {
				if (file == NULL) return -1;
				while (!feof(file)) {
					if (fgets(buffer, 1024, file) == NULL) break;
				}
				infoString = strtok_s(buffer ,"\n", &temp);
				while (infoString != NULL) {
					//strncpy(infoString, buffer, a-buffer+1);
					this->Add(new BaseActuator(&std::string(infoString)));
					infoString = strtok_s(NULL, "\n", &temp);
				}
				fclose(file);
				initialized = true;
			}
			catch (int e) {
				return e;
			}
		}
		*/
		int ActuatorBlock::Init() {
			for (auto a : *actuators) {
				a.second->Init();
			}
		}

		bool ActuatorBlock::DoAction(Action* action)
		{
			for (auto a : actuators_list) {
				if(a->DoAction(action)) return true; //Add safeguards and limits
			}
			return false;
		}

		void ActuatorBlock::Add(Actuator* nextActuator)
		{
			// Add some safety, check insert
			actuators->emplace(nextActuator->GetActuatorType(), nextActuator);
			actuatorCount++;
		}

		void ActuatorBlock::Remove(Actuator* prevActuator)
		{
			//actuators->erase();
			delete prevActuator;
		}

		void ActuatorBlock::Remove(MRS::Device::ActuatorType type)
		{
			// Add some safety
			actuators->erase(type);
			actuatorCount--;
		}

		Actuator* ActuatorBlock::GetActuatorByType(MRS::Device::ActuatorType actionType) {
			//TODO: Change to Action type 
			//TODO: Add dictionary to convert specific action types to actuator types

			switch (actionType) {
			case MRS::Device::ActuatorType::ACTION_ACTUATOR:
				break;
			default:
				if ((actuators->count(actionType) != (size_t)0)) {
					return actuators->find(actionType)->second;
				}
				else return nullptr;
				break;
			}
			return nullptr;
		}

	}
}