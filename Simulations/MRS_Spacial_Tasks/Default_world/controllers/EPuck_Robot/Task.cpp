#include "Task.h"



void Task::SetState(TypeDefinitions::TaskState newState)
{
	state = newState;
}

void Task::SetTimestamp(long time)
{
	timestamp = time;
}

long Task::GetTimestamp()
{
	return timestamp;
}

TypeDefinitions::TaskType Task::GetType()
{
	return type;
}

void Task::SetID(unsigned char id)
{
	ID = id;
}

void Task::SetPriority(float priority)
{
	givenPriority = priority;
}

void Task::SetType(TypeDefinitions::TaskType t)
{
	type = t;
}

void Task::SetStartCondition(Condition * start)
{
	startCondition = start;
}

void Task::SetStartCondition(TypeDefinitions::ConditionType, char *)
{
	std::cout << "This sets condition\n";
}

void Task::SetEndCondition(Condition * end)
{
	executedCondition = end;
}

void Task::SetEndCondition(TypeDefinitions::ConditionType, char *)
{

}

Task::Task() : Task ( 1, 0.9f)
{
	
}

Task::Task(unsigned char id)
{
	SetID(id);
	size = 0;
	actionIterator = 0;
	timestamp = 0;
}

Task::Task(unsigned char id, float priority) : Task (id)
{
	SetPriority(priority);
}


Task::~Task()
{
}

TypeDefinitions::TaskState Task::GetState()
{
	return state;
}

Condition * Task::GetStartCondition()
{
	return startCondition;
}

Condition * Task::GetExecutedCondition()
{
	return executedCondition;
}

float Task::GetPriority()
{
	return givenPriority;
}

int Task::GetActionIterator()
{
	return actionIterator;
}

void Task::SetActionIterator(int it)
{
	actionIterator = it;
}

void Task::GoToNext()
{
	if (actionIterator < size - 1)
		actionIterator++;
	else
		actionIterator = 0;
}

size_t Task::GetSize()
{
	return size;
}

bool Task::IsBuilt()
{
	return (startCondition != nullptr && executedCondition != nullptr && givenPriority != 0); //Add timestamp later
}

unsigned char Task::GetID()
{
	return ID;
}
