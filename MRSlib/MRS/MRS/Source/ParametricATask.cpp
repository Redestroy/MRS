#include "../Include/ParametricATask.hpp"

namespace MRS {
	namespace Task {
		ParametricATask::ParametricATask(unsigned char id, float priority) : Task(id, priority), actions()
		{
			taskSize = actions.begin();
		}

		ParametricATask::ParametricATask(unsigned char id, float priority, Condition* condition) : ParametricATask(id, priority)
		{
			SetStartCondition(condition);
		}

		ParametricATask::ParametricATask(unsigned char id, float priority, Condition* condition, Condition* end_condition) : ParametricATask(id, priority, condition)
		{
			SetEndCondition(end_condition);
		}

		ParametricATask::ParametricATask(std::string description) : Task()
		{
			//use TaskParser?
		}

		ParametricATask::~ParametricATask()
		{

		}

		void ParametricATask::Add(Device::ParametricAction action)
		{
			 //TODO check for memory problems due to scope
			actions.emplace_back(action);
		}

		Device::Action ParametricATask::GetNextCommand()
		{
			if (GetState() != TaskState::IN_PROGRESS) SetState(TaskState::IN_PROGRESS);
			//std::cout << "Task command size: " << size << "\n";
			return actions.at(GetActionIterator());
		}

		std::string* ParametricATask::toString()
		{
			return nullptr;
		}
	}
}