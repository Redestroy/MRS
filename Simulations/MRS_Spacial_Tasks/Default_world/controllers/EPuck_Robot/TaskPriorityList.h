#pragma once
#ifndef TASKPRIORITYLIST
#define TASKPRIORITYLIST
#include "Task.h"
#include "MRS_Robot.h"
#include <vector>
class TaskPriorityList
{
private:
	std::vector<Task *> tasks;
	unsigned char * fullfiled;
	unsigned char size;
	static float GetWeightedPriority(Task *, MRS_Robot *);
public:
	TaskPriorityList();
	TaskPriorityList(char);
	~TaskPriorityList();
	Task* GetNextTask(MRS_Robot *);
	void Add(Task*);
	void Remove(Task*);
	void Remove(int);
	void RemoveById(int);
	int GetSize();
	Task * GetTaskById(int);
	bool IsEmpty() {
		return size == 0;
	}
};
#endif

