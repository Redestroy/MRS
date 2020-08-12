// File:          operator_robot.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Supervisor.hpp>
#include <webots/Emitter.hpp>
#include <webots/Receiver.hpp>
#include <webots/Node.hpp>
#include <string>
#include <fstream>
#include <list>
#include <stdio.h>
#include <iostream>
#include "MessageTranslator.h"
//#define MODE_CBAA
#define MODE_MRS


// All the webots classes are defined in the "webots" namespace
using namespace webots;

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
	int neededparams = 1;
	if (argc != neededparams + 1) return -3;
	char controllerName[255];
	strcpy_s(controllerName, argv[0]);
	const int TASK_SET_ID = std::stoi(argv[1]);
	//const int ROBOT_ID = std::stoi(argv[1]);
	//float originx = std::stof(argv[2]);
	//float originy = std::stof(argv[3]);
	//float origina = std::stof(argv[4]);
	Supervisor *robotOperator = new Supervisor();
	const int TASK_NUMBER = 15;
	const int TimedOut = 20000;
	char buffer[256];
	char * toBuffer;
	std::string temp;
	temp.append("Task set ");
	temp.append(std::to_string(TASK_SET_ID));
	temp.append("\n");
	const std::string HEADER = temp;
	temp.clear();
	temp.append("TaskSets/Taskset");
	temp.append(std::to_string(TASK_SET_ID));
	temp.append(".dat");
	const std::string INPUT_FILE = temp;
	temp.clear();
	temp.append("Results/");
#ifdef MODE_MRS
	temp.append("MRS_RTA");
#endif
#ifdef MODE_CBAA
	temp.append("CBAA");
#endif
	temp.append("/Taskset");
	temp.append(std::to_string(TASK_SET_ID));
	temp.append(".res");
	const std::string OUTPUT_FILE = temp;
	temp.clear();

	const int SECONDS_TO_STEPS = 63; //shange to float later
	const float STEPS_TO_SECONDS = 0.016;

	std::string message, taskString;
	std::list<std::string> taskStrings;
	//std::list<std::string>::iterator taskIterator = taskStrings.begin();
	long int taskTimeline[TASK_NUMBER];
	int i = 0;
	std::fstream myfile;
	myfile.open(INPUT_FILE, std::ios::in);
	if (myfile.is_open()) {
		while (std::getline(myfile, taskString ) && i<TASK_NUMBER) {
			if (myfile.eof())break;
			taskTimeline[i] = SECONDS_TO_STEPS * stoi(taskString.substr(0, taskString.find_first_of(' ')));
			taskStrings.emplace_back(taskString.substr(taskString.find_first_of(' ') + 1));
			std::cout << taskStrings.back() << "\n";
			i++;
		}
	}
	myfile.close();
	int pos = 0;
	
  int timeStep = (int)robotOperator->getBasicTimeStep();

  Emitter *transmitter = robotOperator->getEmitter("emitter");
  Receiver *receptor = robotOperator->getReceiver("receiver");
  Receiver *receptor_serial = robotOperator->getReceiver("receiver_serial");
  transmitter->setChannel(3);
  receptor->setChannel(-1);
  receptor_serial->setChannel(3);
  receptor->enable(timeStep);
  receptor_serial->enable(timeStep);
  

 // if(robotOperator->movieIsReady()) robotOperator->movieStartRecording("movies/test.mp4",480,360,0,20,1,true);
  long int startTimes[TASK_NUMBER+1], endTimes[TASK_NUMBER+1], durations[TASK_NUMBER+1];
  bool taskComplete[TASK_NUMBER], sent[TASK_NUMBER + 1];
  bool GoOutOfLoop = false;
  for (i = 0; i < TASK_NUMBER; i++) {
	  taskComplete[i] = false;
	  startTimes[i] = -1;
	  endTimes[i] = -1;
	  sent[i] = false;
  }
  startTimes[TASK_NUMBER] = -1;
  endTimes[TASK_NUMBER] = -1;
  sent[TASK_NUMBER] = false;
  long int currentTime = 0; // change later
  startTimes[0] = currentTime;
  i = 0;
  std::list<std::string>::iterator iter = taskStrings.begin();
  while (robotOperator->step(timeStep) != -1) {
	  if (!sent[0]) {
		  int k = 0;
		  for (int i = 0; i < TASK_NUMBER; i++) {
			  if ((taskTimeline[i] < currentTime - startTimes[0]) && !sent[i + 1]) {
				  strncpy_s(buffer, iter->c_str(), iter->length());
				  transmitter->send((void *)buffer, iter->length() + 1);
				  iter++;
				  startTimes[i + 1] = currentTime;
				  std::cout << "Sent: " << i << "\n";
				  sent[i+1] = true;
			  }
			  else if (sent[i+1]) k++;
		  }
		  if (k == TASK_NUMBER) sent[0] = true;
	  }
	  while (receptor->getQueueLength() > 0) {
		  int packetSize = receptor->getDataSize();
		  toBuffer = (char *)receptor->getData();
		  strncpy_s(buffer, toBuffer, packetSize);
		  message.append(buffer);
		  std::cout << "Recieved serial: " << message.c_str() << "\n";
#ifdef MODE_CBAA
		  if (MessageTranslator::isValidCBAA(&message, packetSize))

		  {
			  //Message message = createMessage(buffer);
			  std::cout << "Recieved serial is valid\n";
			  switch (MessageTranslator::getMessageTypeCBAA(message[0])) {
			  case TypeDefinitions::CBAA_TASK_COMPLETE_MESSAGE:
				  //std::cout << "Recieved serial is tsk success T: " << !taskComplete[MessageTranslator::getTaskIndex(&message)] <<" \n";
				  if (!taskComplete[MessageTranslator::getTaskIndex(&message)] && sent[MessageTranslator::getTaskIndex(&message) + 1]) {
				  //if (!taskComplete[MessageTranslator::getTaskIndex(&message)] && startTimes[MessageTranslator::getTaskIndex(&message) + 1] != -1) {
					  endTimes[MessageTranslator::getTaskIndex(&message) + 1] = currentTime;
					  taskComplete[MessageTranslator::getTaskIndex(&message)] = true;
					  //std::cout << "Tsk success: " << MessageTranslator::getTaskIndex(&message) << "\n";
				  }
				  break;
			  case TypeDefinitions::CBAA_TASK_MESSAGE:
				  std::cout << "Recieved serial is task\n";
				  // no response, clear buffer
				  break;
			  case TypeDefinitions::CBAA_GPS_REQUEST:
				  std::cout << "Recieved serial is GPS\n";
				  //SendAbs pos response
				  break;
			  case TypeDefinitions::CBAA_BIDS_MESSAGE:
				  // be silent
			  default:
				  std::cout << "Recieved serial is weird\n";
				  //Nothing
				  break;
			  }
		  }
#else
//#elif MODE_MRS
		  if (MessageTranslator::isValid(&message, packetSize)) {
			  //Message message = createMessage(buffer);
			  std::cout << "Recieved serial is valid\n";
			  switch (MessageTranslator::getMessageType(message[0])) {
			  case TypeDefinitions::TASK_COMPLETE_MESSAGE:
				  std::cout << "Recieved serial is tsk success\n";
				  endTimes[MessageTranslator::getTaskIndex(&message) + 1] = currentTime;
				  taskComplete[MessageTranslator::getTaskIndex(&message)] = true;
				  break;
			  case TypeDefinitions::TASK_MESSAGE:
				  std::cout << "Recieved serial is task\n";
				  // no response, clear buffer
				  break;
			  case TypeDefinitions::GPS_REQUEST:
				  std::cout << "Recieved serial is GPS\n";
				  //SendAbs pos response
				  break;
			  default:
				  std::cout << "Recieved serial is weird\n";
				  //Nothing
				  break;
			  }
	  }
#endif // MODE_CBAA
		  else {
			  std::cout << "Recieved serial is invalid\n";
		  }
		  message.erase();
		  receptor->nextPacket();
	  }
	  currentTime++; //right now manually incrament the time
	  GoOutOfLoop = true;
	  
	  if (sent[0]) {
		  std::cout << "\nTask state: ";
		  for (int k = 0; k < TASK_NUMBER; k++) {
			  std::cout << " T: " << k << " ? " << taskComplete[k]<< ";";
		  }
		  std::cout << "\n";
	  }

	  if (sent[0]) {
		  for (int k = 0; k < TASK_NUMBER; k++) {
			  std::cout << "\n T: " << k << " " << taskComplete[k];
			  if (!taskComplete[k]) {
				  GoOutOfLoop = false;
				  break;
			  }
		  }
	  }
	  else {
		  GoOutOfLoop = false;
	  }
	  
	  if (GoOutOfLoop) {
		  std::cout << "Go out of loop \n";
		  break;
	  }
  };
  //End timer - save end time in endTime[0]
  std::cout << "Creating output file ";
  std::cout << OUTPUT_FILE <<"\n";
  try {
	  myfile.open(OUTPUT_FILE, std::ios::out);
  }
  catch (...) {
	  std::cout << "File open error \n";
  }
  endTimes[0] = currentTime;
  std::string output_line;
  output_line.append(HEADER);
  myfile << output_line;
  output_line.clear();
  for (int j = 0; j <= TASK_NUMBER; j++) {
	  durations[j] = endTimes[j] - startTimes[j];
	  output_line.append(std::to_string(j));
	  output_line.append("\t");
	  output_line.append(std::to_string(startTimes[j]*STEPS_TO_SECONDS));
	  output_line.append("\t");
	  output_line.append(std::to_string(endTimes[j]*STEPS_TO_SECONDS));
	  output_line.append("\t");
	  output_line.append(std::to_string(durations[j]*STEPS_TO_SECONDS));
	  output_line.append("\n");
	  myfile << output_line;
	  output_line.clear();
	  //write to file
  }
  myfile.close();
  //Stop recording and save
  //robotOperator->movieStopRecording();
  /*
  while (!robotOperator->movieIsReady()) {
	  if (robotOperator->movieFailed()) {
		  std::cout << "movie failed \n";
	  }
  }*/
  // Enter here exit cleanup code.


  delete robotOperator;
  return 0;
}
