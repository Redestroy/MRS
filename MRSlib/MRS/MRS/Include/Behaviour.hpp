#pragma once
#include "Task.hpp"
namespace MRS {
	namespace Task {
		enum class BehaviourType {
			MOVE_TO_POSITION,
			MOVE_ALONG_A_PATH,
			STANDBY_BEHAVIOUR,
			CLOSED_LOOP_PROCESS,
			WAITING_BEHAVIOUR,
			COORDINATED_ACTION
		};

		class Behaviour : public Task
		{
			Task* baseTask;
			Condition * endCondition;
			BehaviourType behaviour_type;
			std::string behaviourString;
		protected:
			Task* GetBaseTask();
		public:
			Behaviour();
			Behaviour(unsigned char id);
			Behaviour(unsigned char id, float priority);
			Behaviour(unsigned char id, int priority, Task * baseTask);
			Behaviour(std::string * description);
			~Behaviour();

			void SetBaseTask(Task*);
			void SetBehaviorType(BehaviourType);
			void SetEndCondition(Condition * c);
			BehaviourType GetBehaviourType();

			virtual Device::Action GetNextCommand();
			virtual std::string* toString();
			static std::string GetTypename(BehaviourType);
		};
	}
}

