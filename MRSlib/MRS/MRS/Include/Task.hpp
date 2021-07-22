#pragma once
#include "Condition.hpp"
#include "Action.hpp"

namespace MRS {
	namespace Task {
		enum class TaskType
		{
			ATOMIC,
			BEHAVIOUR,
			PARAMETRIC,
			ROLE,
			TREE,
			COMPLEX,
			UNDEFINED
		};

		enum class TaskState
		{
			NEW,
			IDLE,
			QUEUED,
			STARTED,
			IN_PROGRESS,
			SUCCEEDED,
			FAILED
		};

		class Task
		{
		private:
			Condition* startCondition;
			Condition* executedCondition;
			TaskState state;
			int actionIterator;
			TaskType type;

			float givenPriority;
			unsigned char ID;
			long timestamp;
			std::string taskString;
		protected:
			//void SetState(TypeDefinitions::TaskState);
			size_t size;
			void SetID(unsigned char);
			void SetPriority(float);
			void SetType(TaskType);
			virtual void SetStartCondition(Condition*);
			virtual void SetStartCondition(ConditionType, char*);
			virtual void SetEndCondition(Condition*);
			virtual void SetEndCondition(ConditionType, char*);
		public:
			Task();
			Task(unsigned char);
			Task(unsigned char, float);
			~Task();

			TaskState GetState();
			Condition* GetStartCondition();
			Condition* GetExecutedCondition();
			float GetPriority();
			int GetActionIterator();
			void SetActionIterator(int);
			void GoToNext();
			size_t GetSize();
			virtual bool IsBuilt();
			unsigned char GetID();
			void SetState(TaskState);
			void SetTimestamp(long);
			long GetTimestamp();
			TaskType GetType();
			virtual Device::Action GetNextCommand() = 0; // Returns the next command to be executed
			virtual std::string* toString() = 0;

			static TaskState GetTaskStateFromInteger(int);
			static TaskType GetTaskTypeFromInteger(int);
		};
	}
}