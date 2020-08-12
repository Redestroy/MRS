#pragma once
#ifndef TASK
#define TASK
#define DEBUG
#include "TypeDefinitions.h"
#include "Condition.h"
#include "Action.h"
#ifdef DEBUG
#include <iostream>
#endif

class Task
{
private:
	Condition * startCondition;
	Condition * executedCondition;
	TypeDefinitions::TaskState state;
	int actionIterator;
	TypeDefinitions::TaskType type;
	
	float givenPriority;
	unsigned char ID;
	long timestamp;
protected:
	//void SetState(TypeDefinitions::TaskState);
	size_t size;
	void SetID(unsigned char);
	void SetPriority(float);
	void SetType(TypeDefinitions::TaskType);
	virtual void SetStartCondition(Condition *);
	virtual void SetStartCondition(TypeDefinitions::ConditionType, char *);
	virtual void SetEndCondition(Condition *);
	virtual void SetEndCondition(TypeDefinitions::ConditionType, char *);
public:
	Task();
	Task(unsigned char);
	Task(unsigned char, float);
	~Task();

	TypeDefinitions::TaskState GetState();
	Condition * GetStartCondition();
	Condition * GetExecutedCondition();
	float GetPriority();
	int GetActionIterator();
	void SetActionIterator(int);
	void GoToNext();
	size_t GetSize();
	virtual bool IsBuilt();
	unsigned char GetID();
	void SetState(TypeDefinitions::TaskState);
	void SetTimestamp(long);
	long GetTimestamp();
	TypeDefinitions::TaskType GetType();
	virtual Action GetNextCommand() = 0; // Returns the next command to be executed
	virtual std::string * toString() = 0;
};
#endif

