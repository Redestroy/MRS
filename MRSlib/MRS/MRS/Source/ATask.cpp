#include "../Include/ATask.hpp"
namespace MRS {
	namespace Task {
		void ATask::SetCondition(Condition* condition)
		{
			SetStartCondition(condition);

		}

		ATask::ATask() : Task(), commands()
		{
			commands.resize(10);
			taskSize = commands.begin();
			SetType(TaskType::ATOMIC);
		}

		ATask::ATask(size_t size) : Task(), commands()
		{
			commands.resize(size);
			this->taskSize = commands.begin();
		}

		ATask::ATask(size_t size, unsigned char id, float priority) : Task(id, priority), commands()
		{
			commands.resize(size);
			this->taskSize = commands.begin();
		}

		ATask::ATask(size_t size, unsigned char id, float priority, Condition* condition) :ATask(size, id, priority)
		{
			SetCondition(condition);
		}

		ATask::ATask(size_t size, unsigned char id, float priority, Condition* condition, Condition* end) : ATask(size, id, priority, condition)
		{
			SetEndCondition(end);
		}

		ATask::ATask(std::string* taskString) : ATask()
		{
			size_t task_string_size = taskString->length();
			//printf(" this is task");
			this->SetAttributesFromString(taskString, task_string_size);
		}

		ATask::~ATask()
		{
		}

		void ATask::Add(Device::Action action)
		{

			if (size == 0) taskSize = commands.insert(commands.begin(), action);
			else taskSize = commands.insert(taskSize, action);
			taskSize++;
			size++;
		}

		void ATask::SetAttributesFromString(std::string* taskString, size_t size)
		{
			// String example
			// Time to send: not sent or used for log purposes
			// T indicates it is a task
			// 0  - ?
			// 0  - task index
			// T_A - indicates Type of object
			// 0.5 - arg,  here: base weight
			// /C_S Start condition
			// CP2 - 2d position condition
			// 100 100 - args, here: coords in cartesian
			// /C_E End condition
			// CN - Null condition
			// /T_A - ATask specs ('Gcode')
			// 4 - number of actions
			// A_L - action type
			// 0 T 0 0 T_A 0.5 /C_S C_P 100 100 10/C_E C_N/T_A 4 A_L 7HLI A_L 7HLI A_L 7HLI A_L 7HLI
			// Task paramaters
			//std::cout << " Setting args \n";
			//printf(" Setting args \n");
			int currentSpace = 0, nextSpace = 0, firstTemp, secondTemp;
			std::string temp, startConditionSubstring, endConditionSubstring, taskSubstring;
			if ((*taskString)[0] != 'T') return;
			//std::cout << " Task args \n";
			//printf(" Task args \n");
			currentSpace = (int)taskString->find_first_of(' ', taskString->find_first_of('T') + 1);
			nextSpace = (int)taskString->find_first_of(' ', currentSpace + 1);
			try {
				//std::cout << " Id sub " << taskString->substr(currentSpace + 1, nextSpace - currentSpace) << "\n";
				SetID(std::stoi(taskString->substr(currentSpace + 1, nextSpace - currentSpace)));
			}
			catch (...) {
				std::cout << " This is a numeric exception 1 \n";
				printf(" Num error \n");
			}
			currentSpace = nextSpace;
			nextSpace = (int)taskString->find_first_of(' ', currentSpace + 1);
			try {
				//std::cout << " State sub " << taskString->substr(currentSpace + 1, nextSpace - currentSpace) << "\n";
				SetState(Task::GetTaskStateFromInteger(std::stoi(taskString->substr(currentSpace + 1, nextSpace - currentSpace))));// check value
			}
			catch (...) {
				std::cout << " This is a numeric exception 2 \n";
				printf(" Num error 2 \n");
			}

			currentSpace = nextSpace;
			nextSpace = (int)taskString->find_first_of(' ', currentSpace + 1);
			// Add task type switch case
			// Mostly for safety checks
			currentSpace = nextSpace;
			nextSpace = (int)taskString->find_first_of(' ', currentSpace + 1);
			try {
				//std::cout << " Priority sub " << taskString->substr(currentSpace + 1, nextSpace - currentSpace) << "\n";
				SetPriority(std::stof(taskString->substr(currentSpace + 1, nextSpace - currentSpace)));
			}
			catch (...) {
				std::cout << " This is a numeric exception 3 \n";
				printf(" Num error 3 \n");
			}

			//std::cout << "Conditions \n";
			Condition* startCondition, * endCondition;
			firstTemp = (int)(taskString->find_first_of('/'));
			secondTemp = (int)taskString->find_first_of('/', firstTemp + 1);
			startConditionSubstring = taskString->substr(firstTemp, secondTemp - firstTemp);
			firstTemp = secondTemp;
			secondTemp = (int)taskString->find_first_of('/', firstTemp + 1);
			endConditionSubstring = taskString->substr(firstTemp, secondTemp - firstTemp);
			firstTemp = secondTemp;
			//std::cout << " Start " << startConditionSubstring << " \n";
			//std::cout << " End " << startConditionSubstring << " \n";
			//printf(" ATAsk \n");
			taskSubstring = taskString->substr(firstTemp);
			char startType = startConditionSubstring[7];
			char endType = endConditionSubstring[7];
			//TODO: move condition parsing to condition clases and subclasses
			switch (Condition::CharToCondition(startType)) {
			case ConditionType::CONDITION_POSITION:
				//std::cout << " Pos parsing \n";
				//std::cout << " pos string:" << startConditionSubstring << " \n";
				//startCondition = new ConditionPosition(&startConditionSubstring);
				break;
			case ConditionType::CONDITION_NULL:
				//std::cout << " Null parsing \n";
				//startCondition = new ConditionNull();
			default:
				std::cout << " Undefined pos parsing \n";
				std::cout << " Cond char string: " << startType << " \n";
				//startCondition = new ConditionUndefined();
			}
			switch (Condition::CharToCondition(endType)) {
			case ConditionType::CONDITION_POSITION:
				//std::cout << " End pos parsing \n";
				//std::cout << " pos string:"<< endConditionSubstring <<" \n";
				//endCondition = new ConditionPosition(&endConditionSubstring);
				break;
			case ConditionType::CONDITION_NULL:
				//std::cout << " End null parsing \n";
				//endCondition = new ConditionNull();
				break;
			default:
				std::cout << " Undefined end \n";
				std::cout << " Cond char string: " << endType << " \n";
				//endCondition = new ConditionUndefined();
			}
			this->SetStartCondition(startCondition);
			this->SetEndCondition(endCondition);
			//int commandNum = taskSubstring[5] - 48; // change later
			int commandNum = 4;
			try {
				commandNum = stoi(taskSubstring.substr(taskSubstring.find_first_of(' '), taskSubstring.find_first_of(' ', taskSubstring.find_first_of(' ') + 1)));
			}
			catch (...) {
				printf("No number in string %s", taskSubstring.substr(taskSubstring.find_first_of(' '), taskSubstring.find_first_of(' ', taskSubstring.find_first_of(' ') + 1)).c_str());
			}
			if (commandNum >= 0) {
				commands.resize(commandNum + 1);
				taskSize = commands.begin();
			}
			firstTemp = 7;
			//std::cout << " Action creation \n";

			firstTemp = (int)taskSubstring.find_first_of('/', taskSubstring.find_first_of('T'));
			std::string action_string = "";
			for (int i = 0; i < commandNum; i++) {
				secondTemp = (int)taskSubstring.find_first_of('/', firstTemp + 1);
				action_string = taskSubstring.substr(firstTemp + 1, secondTemp - firstTemp - 1);
				this->Add(Device::Action(&action_string));//
				//std::cout << " Action substring " << taskSubstring.substr(firstTemp + 1, secondTemp - firstTemp - 1) << " \n";
				firstTemp = secondTemp;
			}
			this->Add(Device::Action());
		}

		Device::Action ATask::GetNextCommand()
		{
			if (GetState() != TaskState::IN_PROGRESS) SetState(TaskState::IN_PROGRESS);
			//std::cout << "Task command size: " << size << "\n";
			return commands.at(GetActionIterator());
		}

		std::string* ATask::toString()
		{
			//TODO: fix parsing
			std::string temp = "";
			temp.append("Id ");
			temp.append(std::to_string((unsigned int)(GetID())));
			temp.append("State ");
			temp.append(std::to_string((int)GetState()));
			temp.append("\n");
			return &temp; // TODO: replace with actual method
		}
	}

}
/*
void ATask::SetCondition(Condition * condition)
{
	SetStartCondition(condition);
}

ATask::ATask(): Task(),commands()
{
	commands.resize(10);
	taskSize = commands.begin();
}

ATask::ATask(size_t size) : Task(),commands()
{
	commands.resize(size);
	this->taskSize = commands.begin();
}

ATask::ATask(size_t size, unsigned char id, float priority) : Task(id,priority), commands()
{
	commands.resize(size);
	this->taskSize = commands.begin();
}

ATask::ATask(size_t size, unsigned char id, float priority, Condition * condition):ATask(size, id, priority)
{
	SetCondition(condition);
}

ATask::ATask(size_t size, unsigned char id, float priority, Condition * condition, Condition * end) : ATask(size, id, priority, condition)
{
	SetEndCondition(end);
}

ATask::~ATask()
{
}

void ATask::Add(Action action) 
{
	if (size == 0) taskSize = commands.insert(commands.begin(), action);
	else taskSize = commands.insert(taskSize, action);
	taskSize++;
	size++;
}

Action ATask::GetNextCommand()
{
	//if (GetActionIterator() == NULL) return Action();
	if (GetState() != TypeDefinitions::TASK_EXECUTING) SetState(TypeDefinitions::TASK_EXECUTING);
	return commands[GetActionIterator()];
}

std::string * ATask::toString()
{
	std::string * temp = new std::string();
	temp->append("Id ");
	temp->append(std::to_string((unsigned int)(GetID())));
	temp->append("State ");
	temp->append(std::to_string((int)GetState()));
	temp->append("\n");
	return temp; // TODO: replace with actual method
}
*/