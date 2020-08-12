#pragma once
#ifndef ATASK
#define ATASK
#include "Task.h"
#include "Action.h"
#include <Vector> 
#include "ConditionNull.h"
#include "ConditionPosition.h"
#include "ConditionUndefined.h"
#include <iostream>
#include <stdio.h>
class ATask :
	public Task
{
private: 
	std::vector<Action> commands; // Need to think about choosing a data structure - can use an array
	std::vector<Action>::iterator taskSize;
	void SetCondition(Condition *);
public:
	ATask();
	ATask(size_t);
	ATask(size_t, unsigned char, float);
	ATask(size_t, unsigned char, float, Condition *);
	ATask(size_t, unsigned char, float, Condition *, Condition *);
	ATask(std::string *);
	~ATask();
	void Add(Action action);
	void SetAttributesFromString(std::string *, size_t);
	Action GetNextCommand();
	std::string * toString();

};
#endif

