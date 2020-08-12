#include "TaskPriorityList.h"



float TaskPriorityList::GetWeightedPriority(Task * task , MRS_Robot * robot)
{
	return task->GetPriority()*robot->TaskPriority(task);
	
}

TaskPriorityList::TaskPriorityList() : TaskPriorityList(127)
{
}

TaskPriorityList::TaskPriorityList(char maxSize): tasks(maxSize)
{
	fullfiled = new unsigned char[maxSize];
	size = 0;
}


TaskPriorityList::~TaskPriorityList()
{
	delete[] fullfiled;
}

Task * TaskPriorityList::GetNextTask(MRS_Robot * robot)
{
	if (size == 0) return nullptr;
	Task * mostImportant = nullptr;
	float highestPriority = 0;
	bool haveReference = false;
	if (tasks.empty()) return nullptr;
	for (std::vector<Task*>::iterator it = tasks.begin(); it != tasks.end(); it++) {
		if (*it != nullptr) {
			if (!haveReference && ((*it)->GetState() != TypeDefinitions::TASK_COMPLETE && (*it)->GetState() != TypeDefinitions::TASK_FAILED)) {
				mostImportant = *it;
				highestPriority = GetWeightedPriority(mostImportant, robot);
				haveReference = true;
			}
			else if (highestPriority < GetWeightedPriority(*it, robot) && ((*it)->GetState() != TypeDefinitions::TASK_COMPLETE && (*it)->GetState() != TypeDefinitions::TASK_FAILED)) {
				mostImportant = *it;
				highestPriority = GetWeightedPriority(mostImportant, robot);
			}
		}
	}
	return mostImportant;
}

void TaskPriorityList::Add(Task * task)
{
	if (task != nullptr) {
		tasks.at(size) = task;
		fullfiled[size] = (unsigned char)((task->GetID() < 1) | (0b00000001 & (unsigned char)(task->GetState() == TypeDefinitions::SUCCESS)));
		size++;
	}
	else std::cout << "Null task given \n";
}

void TaskPriorityList::Remove(Task * task)
{
	if (task->GetState() != TypeDefinitions::TASK_COMPLETE) task->SetState(TypeDefinitions::TASK_COMPLETE);
	size--;
	/*
	for (std::vector<Task*>::iterator it = tasks.begin(); it != tasks.end(); it++) {
		if (*it == task) {
			tasks.erase(it);
			size--;
			return;
		}
	}*/
	std::cout << "No such task exists within list\n";
}

void TaskPriorityList::Remove(int id)
{
	if (id < size && id >= 0) {
		tasks.erase(tasks.begin() + id);
		size--;
	}
	else std::cout << "id out of bounds\n";
}

void TaskPriorityList::RemoveById(int id)
{
	// check for empty
	if (tasks.empty()) return;
	
	for (std::vector<Task*>::iterator it = tasks.begin(); it != tasks.end(); it++) {
		if ((*it) != nullptr) {
			if ((*it)->GetID() == id) {
				(*it)->SetState(TypeDefinitions::TASK_COMPLETE);
				//size--;
				return;
			}
		}
	}
	/*
	for (std::vector<Task*>::iterator it = tasks.begin(); it != tasks.end(); it++) {
		if ((*it)->GetID() == id) {
			tasks.erase(it);
			size--;
			return;
		}
	}*/
	std::cout << "No such task exists within list\n";
}

int TaskPriorityList::GetSize()
{
	return size;
}

Task * TaskPriorityList::GetTaskById(int id)
{
	if (this->IsEmpty()) return nullptr;
	for (std::vector<Task*>::iterator it = tasks.begin(); it != tasks.end(); it++) {
		if ((*it) == NULL) continue;
		if ((*it)->GetID() == id) {
			return (*it);
		}
	}
	return nullptr;
}


