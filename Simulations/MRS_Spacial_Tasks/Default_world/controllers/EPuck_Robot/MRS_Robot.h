#pragma once
#include "Task.h"
#include "Condition.h"
class MRS_Robot
{
	Task * currentTask;
public:
	MRS_Robot();
	~MRS_Robot();
	//Task methods
	virtual bool startConditionMet(Task *) = 0;
	virtual bool endConditionMet(Task *) = 0;
	virtual float TaskPriority(Task *) = 0;
	virtual int Execute(Task *) = 0;
	virtual int ExecuteTask(Task *) = 0;
	virtual int ExecuteBehaviour(Condition *) = 0;
	virtual int ExecuteBehaviour(TypeDefinitions::ExecutableBehaviour) = 0;
	virtual int ExecuteBehaviour(TypeDefinitions::ExecutableBehaviour, Condition *) = 0;
	//Communication methods
	//virtual int OnMessageRecieved(std::string *, TaskPriorityList *, std::vector<Task> *) = 0;
	virtual int OnTaskRecieved(std::string *) = 0;
	//Task list methods
	virtual int AddTask(Task *) = 0;
	virtual int dumpTask(Task *) = 0;
};

