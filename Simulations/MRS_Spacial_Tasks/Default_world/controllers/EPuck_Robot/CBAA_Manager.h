#pragma once
#include <deque>
#include <vector>
#include <map>
#include "Task.h"
#include "EPuck_R.h"
#include <iostream>
//#include <webots/Robot.hpp>
class CBAA_Manager
{
private:
	std::deque<int> taskIds;
	std::deque<int> robotChanells; 
	std::map<int, Task *> taskMap;
	std::map<int, float> bids;
	std::map<int, bool> myList;	// is made
	//std::map<int, bool> recievedFromChannel;	// is made
	std::string bidsMessage = "";
	std::string handshaker = "";
	std::string buffer = "";
	std::map<int, std::string> bidListsRecieved;
	std::map<int, float> bidsRec;
	static long TIMED_OUT;
	unsigned int bidsRecieved = 0;
	bool haveNoBids = true; //false;
	bool tasksRecieved = false;
	bool allBidsRecieved = false;

	int ROBOT_ID;
	int timestep;
public:
	CBAA_Manager();
	void AuctionPhase(EPuck_R *);
	void PlaceBids(EPuck_R *);
	int ParseBidsMessage(std::string *);
	void ParseBids();
	bool WaitToRecieveBids(EPuck_R *, long);
	bool ConsensusPhase(EPuck_R *);
	void NegotiateConsensus(int);
	long UpdateRobot(EPuck_R *, int, long, long);
	bool ParsePacket(EPuck_R *);
	bool AllComplete();

	bool TA_CBAA(EPuck_R *);

	void SetRobotId(int);
	void ResetTasksRecieved();
	int GetRobotId();
	std::map<int, Task *> * GetTaskMap();
	std::deque<int> * GetTaskIds();
	std::map<int, float> * GetBids();
	std::map<int, bool> * GetMyList();
	void printDebug(int);
	~CBAA_Manager();
};

