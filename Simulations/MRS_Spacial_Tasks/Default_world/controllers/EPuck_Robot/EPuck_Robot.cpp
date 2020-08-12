// File:          EPuck_Robot.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <iostream>
#include <stdio.h>
#include <vector>
#include <string>
#include "EPuckDrive.h"
#include "EpuckLedArray.h"
#include "EpuckEncoderSet.h"
#include "Worldview.h"
#include "EpuckDistanceSensorArray.h"
#include "EPuck_R.h"
#include "CBAA_Manager.h"
//#include "SerialCommunication.h"
//#include "ATask.h"

//#include "MessageTranslator.h"

//#define DEBUGCODE
#define MRS_RTA
//#define CBAA
//
//#include <DirectDrive.h>
//#include <ActuatorBlock.h>

using namespace webots;

int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();
  //robot->getCustomData();
  try {
	  int id = std::stoi(robot->getCustomData());
  }
  catch (...){
	  std::cout << "No custon data in" << robot->getCustomData() <<" \n";
  }
  //
 // std::cout << "1 This far is gud \n";
  
  int neededparams = 4;
  if (argc != neededparams + 1) return -3;
  char controllerName[255];
  strcpy_s(controllerName, argv[0]);
  const int ROBOT_ID = std::stoi(argv[1]);
  float originx = std::stof(argv[2]);
  float originy = std::stof(argv[3]);
  float origina = std::stof(argv[4]);
  ViewPosition2D origin(originx*1000, originy*1000, origina);
  //
 // std::cout << "2 This far is gud \n";
  robot->setMode(webots::Robot::MODE_SIMULATION, nullptr);
  
  Worldview world;
  EPuckDrive *driver = new EPuckDrive(robot);
  EpuckLedArray *leds = new EpuckLedArray(robot);
  EpuckEncoderSet *encoders = new EpuckEncoderSet(robot);
  EpuckDistanceSensorArray *distanceSensors = new EpuckDistanceSensorArray(robot);
  const float TO_MILI = 0.001f;
  std::string buffer;
  //
 // std::cout << "3 This far is gud \n";
  ViewPosition2D goal(500.0f, 500.0f, 3.14f);
  ConditionPosition *goalPosition = new ConditionPosition(&goal, 5);
  
  TaskPriorityList * tasks = new TaskPriorityList(100);
  std::vector<ATask *> taskContainer(100);
  std::vector<ATask *>::iterator taskPlacer = taskContainer.begin();

  ActuatorBlock *actuatorBlock = new ActuatorBlock();
  actuatorBlock->Add(driver);
  actuatorBlock->Add(leds);
  //
  //std::cout << "4 This far is gud \n";
  SensorBlock *sensorBlock = new SensorBlock();
  sensorBlock->Add(encoders);
  sensorBlock->Add(distanceSensors);
  sensorBlock->CreateViews(&world);

  SerialCommunication *comDevice = new SerialCommunication(robot);

  EPuck_R *mRobot = new EPuck_R(robot, originy*1000, originx*1000, origina);
  mRobot->SetActuatorBlock(actuatorBlock);
  mRobot->SetSensorBlock(sensorBlock);
  mRobot->SetWorld(&world);
  mRobot->SetCommunicationDevice(comDevice);
  //
// std::cout << "5 This far is gud \n";
  leds->SetConfiguration(0b10101010);

  int timeStep = 16;//(int)robot->getBasicTimeStep();
  
  long deltaTime = 0, deltatime2, currentTime = 0, previousTime = 0;
#ifdef DEBUGCODE
  CBAA_Manager cbaaManager;
  cbaaManager.SetRobotId(ROBOT_ID);
  while (robot->step(timeStep) != -1) {

	mRobot->GetSensorBlock()->Update(mRobot->GetWorld());
	mRobot->GetWorld()->Update();
	  //mRobot->ExecuteTask(taskList[0]);
	mRobot->ExecuteBehaviour(goalPosition);
	//printf("Num recieved: %d \n", mRobot->GetCommunicationDevice()->GetNumberOfPackets());
	while (mRobot->GetCommunicationDevice()->GetNumberOfPackets() > 0) {
		cbaaManager.ParsePacket(mRobot);
		cbaaManager.AuctionPhase(mRobot);
		if(ROBOT_ID == 1) cbaaManager.printDebug(4);
		if(cbaaManager.ConsensusPhase(mRobot)) cbaaManager.printDebug(3);
		//cbaaManager.printDebug(3);
	}

	//while (mRobot->GetCommunicationDevice()->GetNumberOfPackets() > 0) {
		
	//}
	if (deltatime2 > 200) {
		deltatime2 = 0;
		printf("X: %0.2f\t Y: %0.2f\t A: %0.2f\n", ((ViewPosition2D *)(world.GetPositionView()))->GetX(), ((ViewPosition2D *)(world.GetPositionView()))->GetY(), ((ViewPosition2D *)(world.GetPositionView()))->GetA());
	}
	else {
		deltatime2++;
	}
	  currentTime += timeStep;
  }
#endif // DEBUG
#ifdef MRS_RTA
	Task * activeTask;
	int ReturnCode = 0;
	std::string debugString;
	while (!mRobot->IsEndOfService()) {
		while ((activeTask = tasks->GetNextTask(mRobot)) != nullptr) {
			while (!mRobot->StartConditionMet(activeTask)) {
				debugString.clear();
				debugString.append(std::to_string((char)(activeTask->GetStartCondition()->GetType())));
				//std::cout << "Condition not met: " << debugString << "\n";
				ReturnCode = mRobot->ExecuteBehaviour(activeTask->GetStartCondition());
				if (ReturnCode == -1) {
					mRobot->dumpTask(activeTask);
					if (tasks->IsEmpty()) break; // break to main loop
					activeTask = tasks->GetNextTask(mRobot);
					std::cout << "Unable to reach task start\n";
				}
				// Update
				currentTime += timeStep;
				deltaTime = currentTime - previousTime;
				previousTime = currentTime;
				if (robot->step(timeStep) == -1) {
					mRobot->EndOfService();
					break; // break to main loop
				}
				mRobot->Update(deltaTime);
				// Communication
				while (mRobot->GetCommunicationDevice()->GetNumberOfPackets() > 0) {
					//std::cout << "Recieved" << mRobot->GetCommunicationDevice()->ReturnLastPacket() << "\n";
					buffer.append(mRobot->GetCommunicationDevice()->ReturnLastPacket()->c_str());
					//printf("Recieved: %s \n", (buffer.c_str()));
					mRobot->OnMessageRecieved(&buffer, tasks);
					activeTask = tasks->GetNextTask(mRobot);
					buffer.erase();
					if (robot->step(timeStep) == -1) {
						mRobot->EndOfService();
						break; // break to main loop
					}
				}
			}
			mRobot->ExecuteBehaviour(TypeDefinitions::BEHAVIOUR_SAFE_WAIT);
			// Excecute task
			if(activeTask != nullptr) ReturnCode = mRobot->ExecuteTask(activeTask);
			//ReturnCode = 1;
			//std::cout << ReturnCode << " ret \n";
			if (ReturnCode == -1) mRobot->dumpTask(activeTask);
			if (ReturnCode != 0) {
				tasks->Remove(activeTask);
				activeTask = nullptr;
			}
			//std::cout << "ATask finished: \n";
			// Update
			ReturnCode = robot->step(timeStep);
			std::cout << ReturnCode << " ret \n";
			ReturnCode = robot->step(timeStep);
			if (ReturnCode == -1) {
				std::cout << "Work done: \n";
				mRobot->EndOfService();
				break; // break to main loop
			}
			currentTime += timeStep;
			deltaTime = currentTime - previousTime;
			previousTime = currentTime;
			//std::cout << "Gonna update: \n";
			mRobot->Update(activeTask, deltaTime);
			//std::cout << "Steped: \n";
			
			// Communication
			while (mRobot->GetCommunicationDevice()->GetNumberOfPackets() > 0) {
				buffer.append(mRobot->GetCommunicationDevice()->ReturnLastPacket()->c_str());
				printf("Recieved: %s \n", (buffer.c_str()));
				mRobot->OnMessageRecieved(&buffer, tasks);
				activeTask = tasks->GetNextTask(mRobot);
				buffer.erase();
				if (robot->step(timeStep) == -1) {
					mRobot->EndOfService();
					break; // break to main loop
				}
			}
			std::cout << "After com: \n";
			//if (robot->step(timeStep) == -1) {
			//	std::cout << "Work done: \n";
			//	mRobot->EndOfService();
			//	break; // break to main loop
			//}
			currentTime += timeStep;
			deltaTime = currentTime - previousTime;
			previousTime = currentTime;
			//std::cout << "Gonna update: \n";
			mRobot->Update(activeTask, deltaTime);
			//std::cout << "Updated: \n";
			
		}
		std::cout << "Gonna do standby: \n";
		if (activeTask == NULL) mRobot->ExecuteBehaviour(TypeDefinitions::BEHAVIOUR_STANDBY);
		// Update
		if (robot->step(timeStep) == -1) {
			mRobot->EndOfService();
			//break; // break to main loop
		}

		// Communication
		while (mRobot->GetCommunicationDevice()->GetNumberOfPackets() > 0) {
				buffer.append(mRobot->GetCommunicationDevice()->ReturnLastPacket()->c_str());
				printf("Recieved: %s \n", (buffer.c_str()));
				mRobot->OnMessageRecieved(&buffer, tasks);
				activeTask = tasks->GetNextTask(mRobot);
				buffer.erase();
				if (robot->step(timeStep) == -1) {
					mRobot->EndOfService();
					break; // break to main loop
				}
		}
		currentTime += timeStep;
		deltaTime = currentTime - previousTime;
		previousTime = currentTime;
		mRobot->Update(activeTask, deltaTime);
  }
#endif // MRS-RTA

#ifdef CBAA
	  CBAA_Manager cbaaManager;
	  cbaaManager.SetRobotId(ROBOT_ID);
	  bool validBids = false;
	  int ReturnCode = 0;
	  while (robot->step(timeStep) != -1 && !(mRobot->IsEndOfService())) {
		  //std::cout << "In Main loop \n";

		  // if packets>0
		  validBids = cbaaManager.TA_CBAA(mRobot);
		  
		  if(!(cbaaManager.GetTaskMap()->empty()) && !cbaaManager.AllComplete() && validBids)	{
			  for (const auto& taskId : *(cbaaManager.GetTaskIds())) {
				  if (cbaaManager.GetMyList()->at(taskId) && cbaaManager.GetTaskMap()->at(taskId)->GetState()!=TypeDefinitions::TASK_COMPLETE) {
					  while (!mRobot->StartConditionMet(cbaaManager.GetTaskMap()->at(taskId)) && validBids) {
						  ReturnCode = mRobot->ExecuteBehaviour(cbaaManager.GetTaskMap()->at(taskId)->GetStartCondition());
						  
						  currentTime = previousTime = cbaaManager.UpdateRobot(mRobot, timeStep, currentTime, previousTime);
						  // Communication
						  if (mRobot->GetCommunicationDevice()->GetNumberOfPackets() > 0) {
							  validBids = false;
						  }
					  }
					  if (!validBids) break;
					  ReturnCode = mRobot->ExecuteTask(cbaaManager.GetTaskMap()->at(taskId));
					  // Updates
					  currentTime = previousTime = cbaaManager.UpdateRobot(mRobot, timeStep, currentTime, previousTime);
					  // Communication
					  if (mRobot->GetCommunicationDevice()->GetNumberOfPackets() > 0) {
						  validBids = false;
						  break;
					  }
				  }  
			  }
			  if (cbaaManager.AllComplete()) std::cout << " Tasks complete \n";
			  //else std::cout << " Tasks not complete \n";
		  }

		  else {
			  mRobot->ExecuteBehaviour(TypeDefinitions::BEHAVIOUR_STANDBY);
			  currentTime = previousTime = cbaaManager.UpdateRobot(mRobot, timeStep, currentTime, previousTime);
			  if (mRobot->GetCommunicationDevice()->GetNumberOfPackets() > 0) {
				  validBids = false;
			  }
		  }
	  }
#endif // CBAA
  // Enter here exit cleanup code.
 // delete
  delete robot;
  return 0;
}


