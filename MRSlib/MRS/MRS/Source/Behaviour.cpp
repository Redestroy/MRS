#include "../Include/Behaviour.hpp"
namespace MRS {
	namespace Task {
		Task* Behaviour::GetBaseTask()
		{
			return baseTask;
		}
		Behaviour::Behaviour() : Task()
		{
		}

		Behaviour::Behaviour(unsigned char id) : Task(id)
		{
		}

		Behaviour::Behaviour(unsigned char id, float priority) : Task(id, priority)
		{
		}

		Behaviour::Behaviour(unsigned char id, int priority, Task* baseTask) : Behaviour(id, priority)
		{
			this->baseTask = baseTask; //TODO: use smart pointers
		}

		Behaviour::Behaviour(std::string* taskString) : Behaviour()
		{
			// parse behaviour using task parser
			size_t task_string_size = taskString->length();
			//printf(" this is task");
			//TaskParser::SetAttributesFromString(taskString, task_string_size, this);
			// TODO: create failed/unknown task that is created when parsing errors occur
		}


		Behaviour::~Behaviour()
		{
		}

		void Behaviour::SetBaseTask(Task* task)
		{
			baseTask = task;
		}

		void Behaviour::SetBehaviorType(BehaviourType type)
		{
			behaviour_type = type;
		}

		void Behaviour::SetEndCondition(Condition* c)
		{
			endCondition = c;
		}

		BehaviourType Behaviour::GetBehaviourType()
		{
			return behaviour_type;
		}

		Device::Action Behaviour::GetNextCommand()
		{
			if (GetState() == TaskState::FAILED || GetState() == TaskState::SUCCEEDED) return Device::Action(); //complete
			if (baseTask->GetNextCommand().GetActionType() == Device::ActionType::ACTION_NULL) {
				baseTask->SetActionIterator(0);
			}
			return baseTask->GetNextCommand();
		}

		std::string* Behaviour::toString()
		{
			behaviourString = "TODO: add proper parsing";
			return &behaviourString;
		}

		std::string Behaviour::GetTypename(BehaviourType type)
		{
			switch (type) {
			case BehaviourType::MOVE_TO_POSITION:
				return "MOVE_TO_POSITION";
			case BehaviourType::MOVE_ALONG_A_PATH:
				return "MOVE_ALONG_A_PATH";
			case BehaviourType::STANDBY_BEHAVIOUR:
				return "STANDBY_BEHAVIOUR";
			case BehaviourType::CLOSED_LOOP_PROCESS:
				return "CLOSED_LOOP_PROCESS";
			case BehaviourType::WAITING_BEHAVIOUR:
				return "WAITING_BEHAVIOUR";
			case BehaviourType::COORDINATED_ACTION:
				return "COORDINATED_ACTION";
			}
		}
	}
}
