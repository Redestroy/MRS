#include "EPuck_R.h"

#include <iostream>
#include <stdio.h>



EPuck_R::EPuck_R() : EPuck_R(nullptr)
{
}

EPuck_R::EPuck_R(webots::Robot * rob) : EPuck_R(rob, 0.0f, 0.0f, 0.0f)
{
	
}

EPuck_R::EPuck_R(webots::Robot * rob, float x, float y, float a) : MRS_Robot(), taskContainer()
{
	robot = rob;
	endOfService = false;
	origin = new ViewPosition2D(x, y, a);
	A1 = 0.0003f;
	A2 = 3000.0f;
	timeCounter = 0;
	divider = 1000;
	emptyCycles = 0;
	startDelay = -1;
}


EPuck_R::~EPuck_R()
{
	delete origin;
}

void EPuck_R::Update(long deltatime)
{
	sensors->Update(world);
	world->Update();
	timeCounter += deltatime;
}

void EPuck_R::Update(Task * task, long deltatime)
{
	this->Update(deltatime);
}

void EPuck_R::ResetTimer()
{
	timeCounter = 0;
}

long EPuck_R::GetTime()
{
	return (long)((double)timeCounter/divider);
}

long EPuck_R::GetTimeCounter()
{
	return timeCounter;
}

bool EPuck_R::startConditionMet(Task *task)
{
	if (world == nullptr) return false;
	return task->GetStartCondition()->isMet(this->world);
}

bool EPuck_R::endConditionMet(Task * task)
{
	if (world == nullptr) return false;
	if (task->GetExecutedCondition()->GetType() == TypeDefinitions::CONDITION_NULL) return task->GetState() == TypeDefinitions::TASK_COMPLETE;
	return task->GetStartCondition()->isMet(this->world);
}
int EPuck_R::Execute(Task * task)
{
//#define HARDCODED
//#ifdef HARDCODED
	//return 1;
//#else
	ATask * temp = (ATask *)task;
	Action * nextAction = nullptr;
	bool bad = false;
	try {
		nextAction = new Action(temp->GetNextCommand());
	}
	catch (...) {
		std::cout << "Exception: \n";
		bad = true;
	}

	if (bad) return -1;

	if (nextAction != nullptr) {
		if (actuators->GetActuatorByType(nextAction->GetActionType()) != nullptr) {
			actuators->GetActuatorByType(nextAction->GetActionType())->doAction(nextAction);
			return 1;
			//std::cout << "doing action\n";
		}

		else if (nextAction->GetActionType() == TypeDefinitions::NULL_ACTUATOR) {
			std::cout << "Finished" << std::to_string(task->GetID()) << "\n";
			task->SetState(TypeDefinitions::TASK_COMPLETE);
			return 2;
		}
		else if (nextAction->GetActionType() == TypeDefinitions::DELAY) {
			if (startDelay == -1) startDelay = timeCounter;
			else if (timeCounter - startDelay > nextAction->GetActionParameter()) std::cout << "waiting " << std::to_string(task->GetID()) << " Time left: " << nextAction->GetActionParameter() - timeCounter + startDelay << " Start delay "<< startDelay << " Param "<< nextAction->GetActionParameter() << "\n";
			else {
				startDelay = -1;
				return 1;
			}
			return 0;
		}
	}
	else {
		std::cout << "Task cannot be executed\n";
		task->SetState(TypeDefinitions::TASK_FAILED);
		return -1;
	}
	return 0;

//#endif // HARDCODED
}

int EPuck_R::ExecuteTask(Task * task)
{
	int ReturnCode = -1;
	int doNext = 0;
	emptyCycles = 0;
	double timestepOrig = this->GetRobot()->getBasicTimeStep();
	int timestep = (int)(this->GetRobot()->getBasicTimeStep()); //this is double?
	while (task->GetActionIterator() < (int)task->GetSize()) {
		doNext = this->Execute(task);
		if (this->checkTaskCompletion(task) == 1) {
			OnTaskComplete(task->GetID(), true);
			std::cout << "Returning \n";
			return 1;
		}
		if (doNext<0 || this->checkTaskCompletion(task) == -1) return -1;
		if (doNext == 1) task->GoToNext();//task->SetActionIterator(task->GetActionIterator()+1);
		if (this->GetRobot()->step(timestep) == -1) return -1;
		Update(timestep);
	}
	return ReturnCode;
}

int EPuck_R::ExecuteBehaviour(Condition * start)
{
	if (start == nullptr) {
		printf("Task not created properly");
		return -1;
	}
	switch (start->GetType()) {
	case TypeDefinitions::CONDITION_POSITION:
		return ExecuteBehaviour(TypeDefinitions::BEHAVIOUR_SAFE_DRIVE, start);
		break;
	case TypeDefinitions::CONDITION_NULL:
		return 1;
	default:
		return -1;
	}
	return 0;
}

int EPuck_R::ExecuteBehaviour(TypeDefinitions::ExecutableBehaviour behaviour)
{
	switch (behaviour) {
	case TypeDefinitions::BEHAVIOUR_STANDBY:
		if (emptyCycles < 800000) {
			//std::cout << "STANDBY CYCLES TILL COMPLETION: " << 8000 - emptyCycles << "\n";
			ExecuteBehaviour(TypeDefinitions::BEHAVIOUR_SAFE_WAIT);
			emptyCycles++;
		}
		else {
			this->EndOfService();
			std::cout << "EXITING" << 8000 - emptyCycles << "\n";
		}
		break;
	case TypeDefinitions::BEHAVIOUR_SAFE_WAIT:
		//std::cout << "Rest \n";
		//((DirectDrive *)(actuators->GetActuatorByType(TypeDefinitions::DIFFERENTIAL_DRIVE)))->Forward(0); // Replace with evasion using sensors
		Behaviour_SafeDrive(((ViewPosition2D *)(this->GetWorld()->GetPositionView()))->GetX(), ((ViewPosition2D *)(this->GetWorld()->GetPositionView()))->GetY(), ((ViewPosition2D *)(this->GetWorld()->GetPositionView()))->GetA(), 0.001f);
		return 1;
	default:
		return 0;
	}
	return 1;
}

int EPuck_R::ExecuteBehaviour(TypeDefinitions::ExecutableBehaviour behaviour, Condition * condition)
{
	switch (behaviour) {
	case TypeDefinitions::BEHAVIOUR_SAFE_DRIVE:
		emptyCycles = 0;
		float x, y, a;
		x = ((ConditionPosition *)(condition))->GetX();
		y = ((ConditionPosition *)(condition))->GetY();
		a = ((ConditionPosition *)(condition))->GetA();
		//std::cout << "Safe drive to: x " << x << " y " << y << " a " << a << " with tolerance " << ((ConditionPosition *)(condition))->GetPrecision() << "\n";
		return this->Behaviour_SafeDrive(x, y, a, ((ConditionPosition *)(condition))->GetPrecision());
	}
	return 0;
}

int EPuck_R::OnMessageRecieved(std::string * buffer, TaskPriorityList * tasks)
{	
		printf("Recieved: %s \n", (buffer->c_str()));
		if (MessageTranslator::isValid(buffer, buffer->length())) {
			//Message message = createMessage(buffer);
			printf("Is valid \n");
			Task * temp;
			switch (MessageTranslator::getMessageType((*buffer)[0])) {
			case TypeDefinitions::TASK_COMPLETE_MESSAGE:
				//remove task from TaskList
				//tasks->RemoveById(MessageTranslator::getTaskIndex(buffer));
				tasks->RemoveById(MessageTranslator::getTaskIndex(buffer));
				break;
			case TypeDefinitions::TASK_MESSAGE:
				//add task to tasklist if no such task is present
				//printf("Creating task\n");
				if (tasks->GetTaskById(MessageTranslator::getTaskIndex(buffer)) == nullptr) {
					temp = new ATask(buffer);
					taskContainer.emplace_back(temp);
					tasks->Add((Task*)taskContainer.back());
				}
				else printf("task not unique\n");
				// Later add switch for different task types
				break;
			case TypeDefinitions::GPS_REQUEST:
				//SendAbs pos response
				break;
			default:
				//Nothing
				break;
			}
		}
		else
		{
			printf("Invalid message \n");
			return -1;
		}
}

int EPuck_R::OnMessageRecievedCBAA(std::string * buffer, std::map<int, Task*>* tasks)
{
	//printf("Recieved: %s \n", (buffer->c_str()));
	if (MessageTranslator::isValidCBAA(buffer, buffer->length())) {
		//Message message = createMessage(buffer);
		//printf("Is valid \n");
		Task * temp;
		switch (MessageTranslator::getMessageTypeCBAA((*buffer)[0])) {
		case TypeDefinitions::CBAA_TASK_COMPLETE_MESSAGE:
			//remove task from TaskList
			try {
				tasks->at(MessageTranslator::getTaskIndex(buffer))->SetState(TypeDefinitions::TASK_COMPLETE);
				//printf("Another task from the task list was completed\n");
			}
			catch (...) {
				return -1; // No such task was ever given
			}
			return -4;//MessageTranslator::getTaskIndex(buffer);
			break;
		case TypeDefinitions::CBAA_TASK_MESSAGE:
			//add task to tasklist if no such task is present
			//printf("Creating task\n");
			
			if (tasks->find(MessageTranslator::getTaskIndex(buffer)) == tasks->end()) {
				temp = new ATask(buffer);
				taskContainer.emplace_back(temp);
				//tasks->Add((Task*)taskContainer.back());
				tasks->emplace(MessageTranslator::getTaskIndex(buffer), temp);
				return MessageTranslator::getTaskIndex(buffer);
			}
			else printf("task not unique\n");
			break;
		case TypeDefinitions::CBAA_HANDSHAKE:
			// switch channel to other comms
			
			break;
		case TypeDefinitions::CBAA_BIDS_MESSAGE:
			// if a bids message is recieved, add to bidsRec
			//printf("Bids recieved\n");
			return -3;
			break;
		case TypeDefinitions::CBAA_GPS_REQUEST:
			//SendAbs pos response
			break;
		default:
			//Nothing
			break;
		}
		return -1;
	}
	else
	{
		printf("Invalid message \n");
		return -1;
	}
}

int EPuck_R::OnTaskRecieved(std::string *)
{
	return 0;
}

int EPuck_R::AddTask(Task *)
{
	return 0;
}

int EPuck_R::dumpTask(Task *)
{
	return 0;
}

float EPuck_R::TaskPriority(Task * task)
{
	if (!task->IsBuilt()) return 0;
	if (task->GetStartCondition()->GetType() == TypeDefinitions::CONDITION_POSITION) {
		
		float priority = task->GetPriority()*(A1 * (this->GetTime() - task->GetTimestamp()) + A2 / distance(
			((ConditionPosition *)(task->GetStartCondition()))->GetX(),
				((ConditionPosition *)(task->GetStartCondition()))->GetY(),
				((ViewPosition2D *)(world->GetPositionView()))->GetX(),
				((ViewPosition2D *)(world->GetPositionView()))->GetY()
			)); 
		//std::cout << " Task priority : " << priority << "\n";
		//std::cout << " Time priority : " << A1 * (this->GetTime()) << "\n";
		/*std::cout << " Distance priority : " << A2 / distance(
			((ConditionPosition *)(task->GetStartCondition()))->GetX(),
			((ConditionPosition *)(task->GetStartCondition()))->GetY(),
			((ViewPosition2D *)(world->GetPositionView()))->GetX(),
			((ViewPosition2D *)(world->GetPositionView()))->GetY()
		) << "\n";*/
		//std::cout << " Base priority : " << task->GetPriority() << "\n";
		return task->GetPriority()*(A1 * (this->GetTime() - task->GetTimestamp())
			+ A2 / distance(
			((ConditionPosition *)(task->GetStartCondition()))->GetX(),
			((ConditionPosition *)(task->GetStartCondition()))->GetY(),
			((ViewPosition2D *)(world->GetPositionView()))->GetX(),
			((ViewPosition2D *)(world->GetPositionView()))->GetY()
		)) ; // A1*time + A2 / distance
	}
	else {
		return -1;
	}
}

int EPuck_R::checkTaskCompletion(Task * task)
{
	if (task->GetState() == TypeDefinitions::TASK_COMPLETE) return 1;
	if (task->GetState() == TypeDefinitions::TASK_FAILED) return -1;
	if (task->GetExecutedCondition()->isMet(world)) {
		if (task->GetExecutedCondition()->GetType() == TypeDefinitions::CONDITION_NULL) {
			//return 1;
			if (task->GetNextCommand().GetActionType() == TypeDefinitions::NULL_ACTUATOR) {
				task->SetState(TypeDefinitions::TASK_COMPLETE);
				return 1;
			}
			else {
				task->SetState(TypeDefinitions::TASK_EXECUTING);
				return 0;
			}
		}
		else {
			task->SetState(TypeDefinitions::TASK_COMPLETE);
			return 1;
		}

	}
	if (task->GetState() == TypeDefinitions::TASK_EXECUTING) return 0;
	if (task->GetActionIterator() > 0) {
		task->SetState(TypeDefinitions::TASK_EXECUTING);
		return 1;
	}
	return -1;
}

bool EPuck_R::StartConditionMet(Task * task)
{
	if (task != nullptr) {
		return task->IsBuilt() && task->GetStartCondition()->isMet(world);
	}
	return true;
}

bool EPuck_R::EndConditionMet(Task * task)
{
	return task->GetStartCondition()->isMet(world);
}

bool EPuck_R::IsEndOfService()
{
	return endOfService;
}

void EPuck_R::EndOfService()
{
	std::cout << "Work done \n";
	endOfService = true;
}

void EPuck_R::OnTaskComplete(int id, bool success)
{
	char cmd[10];
	sprintf_s(cmd, "E: %d %d\0", id, 1);
	//if (comDevice->GetChannel() != 3) comDevice->SwitchChannel(3);
	//if (comDevice->GetChannel() != -1) comDevice->SwitchChannel(-1);
	comDevice->SendMessage(cmd, (int)strlen(cmd));
	std::cout << "Sent message: " << cmd << '\n';
	//if (comDevice->GetChannel() != 3) comDevice->SwitchChannel(3);
	//if(robot->step(4)==-1) return;
	return;
}



int EPuck_R::Behaviour_SafeDrive(float x, float y, float a, float precision) // should be a seperate precision for angle, fix later
{
	DirectDrive * driver = (DirectDrive *)(this->GetActuatorBlock()->GetActuatorByType(TypeDefinitions::DIFFERENTIAL_DRIVE));
	double dx, dy, da, ang;
	double precisionAng = 0.1;
	double precisionAngFinal = 3.14 * precision / 100.0f;
	dx = x - ((ViewPosition2D *)(world->GetPositionView()))->GetX();
	dy = y - ((ViewPosition2D *)(world->GetPositionView()))->GetY();
	ang = ((ViewPosition2D *)(world->GetPositionView()))->GetA();
	int signature = 1;
	if (a < ang && abs(a - ang) > 3.14159 || a > ang && abs(a - ang) < 3.14159) signature = -1;
	da = (signature)*(fmin(abs(a - ang), 6.28318 - abs(a - ang)));
	float sinAdjust = world->GetAverageDistanceSin(); // Force limiting linear speed
	float cosAdjust = world->GetAverageDistanceCos(); // force limiting rotational speed
	//std::cout << "Sin adj: " << sinAdjust << " Cos adj:" <<cosAdjust << "\n";
	
	//ang += 6.28318;
	//ang = fmod(ang, 6.28318);
	// maybe while
	if (abs(dx) < precision &&
		abs(dy) < precision &&
		abs(da) < precisionAngFinal){
		driver->Forward(0);
		//std::cout << "Stop" << "\n";
		return 1;
	}
	else if (abs(dx) < precision &&
		abs(dy) < precision) {
		float propA = 0.6f, propMod = 3, MAX_SPEED = 6.27f;
		if (da < 0) {
			driver->CounterClokwise((float)(fmin(propA*abs(da), MAX_SPEED)));
			//std::cout << "CC: " << propA * abs(da) << "\n";
		}
		else {
			driver->Clockwise((float)(fmin(propA*abs(da), MAX_SPEED)));
			//std::cout << "C: " << propA * abs(da) << "\n";
		}
		return 1;
	}
	else {
		//calculate vector
		float vectorMod = (float)sqrt(dx*dx + dy*dy);
#define Y_FIRST
#ifdef Y_FIRST
		float avector = (float)atan2(dy, dx);
		//std::cout << "A_VEC: " << avector << "\n";
#else
		float avector = atan2(dx, dy);
#endif
		bool vecDir = (dy >= 0);
		//if (!vecDir) avector += 3.14159;
		
		float propA = 1.5915f, propMod = 3.0f, MAX_SPEED = 6.27f, propSin = 0.3f, propCos = 0.1f;
		if (ang >= (avector - precisionAng) && ang <= (avector + precisionAng) && vecDir) {
			//if pointing in the direction, forward
			if(vecDir) driver->Forward((float)fmax(fmin(propMod*vectorMod, MAX_SPEED) - sinAdjust*propSin, 0)); //same as below but without rotation speed adjust
			else driver->Backward((float)fmax((fmin(propMod*vectorMod, MAX_SPEED) + sinAdjust * propSin, 0), MAX_SPEED));																			   //std::cout << "F: " << fmin(propMod*vectorMod, MAX_SPEED) << "\n";
		}
		else {
			float deltaSpeed = (float)fmin(abs(avector - ang)*propA, MAX_SPEED); // calc rotational speed
			float maxSpeed = (float)fmin(deltaSpeed + vectorMod * propMod, MAX_SPEED); // calc max speed for wheel
			float minspeed = (float)fmax(0, maxSpeed - deltaSpeed); // calc min speed for wheel
			
			//std::cout << "L: " << minspeed << " R: " << maxSpeed << "\n";
			//if (vecDir) {
			if (avector > ang && abs(avector - ang) < 3.14159 || avector < ang && abs(avector - ang) > 3.14159) {
				minspeed = minspeed / 2.0f;
				maxSpeed = maxSpeed / 2.0f;
				minspeed -= sinAdjust * propSin;
				maxSpeed += cosAdjust * propCos;
				maxSpeed = constrain(maxSpeed, -MAX_SPEED, MAX_SPEED);
				minspeed = constrain(minspeed, -MAX_SPEED, MAX_SPEED);
				driver->Arc(minspeed, maxSpeed); // have arc in correct direction
			}
			else {
				minspeed = minspeed / 2.0f;
				maxSpeed = maxSpeed / 2.0f;
				minspeed -= sinAdjust * propSin;
				maxSpeed -= cosAdjust * propCos;
				maxSpeed = constrain(maxSpeed, -MAX_SPEED, MAX_SPEED);
				minspeed = constrain(minspeed, -MAX_SPEED, MAX_SPEED);
				driver->Arc(maxSpeed, minspeed);
			}
			//}
			//else {
			//	if (avector > ang)driver->Arc(maxSpeed, minspeed); // have arc in correct direction
			//	else driver->Arc(minspeed, maxSpeed);
			//}
		}
	}
	return 0;
}

// Getters / Setters
webots::Robot * EPuck_R::GetRobot()
{
	return robot;
}

void EPuck_R::SetRobot(webots::Robot * rob)
{
	robot = rob;
}

ActuatorBlock * EPuck_R::GetActuatorBlock()
{
	return actuators;
}

void EPuck_R::SetActuatorBlock(ActuatorBlock * actu)
{
	actuators = actu;
}

SensorBlock * EPuck_R::GetSensorBlock()
{
	return sensors;
}

void EPuck_R::SetSensorBlock(SensorBlock * sense)
{
	sensors = sense;
}

Worldview * EPuck_R::GetWorld()
{
	return world;
}

void EPuck_R::SetWorld(Worldview * worldv)
{
	world = worldv;
	world->SetOrigin(origin);
}

SerialCommunication * EPuck_R::GetCommunicationDevice()
{
	return comDevice;
}

void EPuck_R::SetCommunicationDevice(SerialCommunication * com)
{
	comDevice = com;
}

Task * EPuck_R::GetTask(int id)
{
	return taskContainer.at(id);
}

float EPuck_R::distance(float x1, float y1, float x2, float y2)
{
	float dx = x1 - x2;
	float dy = y1 - y2;
	return (sqrt(dx*dx + dy*dy));
}

float EPuck_R::constrain(float x, float min, float max)
{
	return fmax(fmin(x,max),min);
}
