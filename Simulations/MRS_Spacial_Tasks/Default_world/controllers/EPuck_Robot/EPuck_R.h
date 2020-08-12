#pragma once
#include <webots/Robot.hpp>
#include <deque>
#include "ActuatorBlock.h"
#include "SensorBlock.h"
#include "SerialCommunication.h"
#include "Worldview.h"
#include "DirectDrive.h"
#include "ConditionPosition.h"
#include "MRS_Robot.h"
#include "ATask.h"
#include <math.h>
#include "MessageTranslator.h"
#include "TaskPriorityList.h"
class EPuck_R : public MRS_Robot
{
	//
	webots::Robot * robot;
	ActuatorBlock * actuators;
	SensorBlock * sensors;
	SerialCommunication * comDevice;
	Worldview * world;
	std::deque<Task *> taskContainer;
	bool endOfService;
	float A1, A2;
	int emptyCycles;
	long timeCounter;
	long divider;
	long startDelay;
	//int timestep;
public:
	View * origin;
	EPuck_R();
	EPuck_R(webots::Robot *);
	EPuck_R(webots::Robot *, float, float, float);
	~EPuck_R();
	// System
	void Update(long);
	void Update(Task *, long);
	void ResetTimer();
	long GetTime();
	long GetTimeCounter();
	// MRS robot stuff
	bool startConditionMet(Task *); //N
	bool endConditionMet(Task *); //N
	int Execute(Task *); //N
	int ExecuteTask(Task *); //N
	int ExecuteBehaviour(Condition *); //N
	int ExecuteBehaviour(TypeDefinitions::ExecutableBehaviour); //N
	int ExecuteBehaviour(TypeDefinitions::ExecutableBehaviour, Condition *); //N
	//Communication methods
	int OnMessageRecieved(std::string *, TaskPriorityList *); //N
	int OnMessageRecievedCBAA(std::string *, std::map<int, Task *> * tasks); //N
	int OnTaskRecieved(std::string *); //N
	//Task list methods
	int AddTask(Task *); //N
	int dumpTask(Task *); //N
	float TaskPriority(Task *);
	int checkTaskCompletion(Task *);
	bool StartConditionMet(Task *);
	bool EndConditionMet(Task *);
	bool IsEndOfService();
	void EndOfService();
	void OnTaskComplete(int, bool);

	//Beheviour methods
	int Behaviour_SafeDrive(float, float, float, float);

	// Setters/Getters
	webots::Robot * GetRobot();
	void SetRobot(webots::Robot *);
	ActuatorBlock * GetActuatorBlock();
	void SetActuatorBlock(ActuatorBlock *);
	SensorBlock * GetSensorBlock();
	void SetSensorBlock(SensorBlock *);
	Worldview * GetWorld();
	void SetWorld(Worldview *);
	SerialCommunication * GetCommunicationDevice();
	void SetCommunicationDevice(SerialCommunication *);
	Task * GetTask(int);
	static float distance(float, float, float, float);
	static float constrain(float, float, float);

};

