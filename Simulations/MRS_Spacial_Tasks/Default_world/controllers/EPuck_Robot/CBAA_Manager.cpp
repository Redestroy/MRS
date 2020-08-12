#include "CBAA_Manager.h"

long CBAA_Manager::TIMED_OUT = 1000;

CBAA_Manager::CBAA_Manager() : taskIds(), robotChanells(), taskMap(), bids(), myList(), bidListsRecieved(), bidsRec()
{
	std::cout << "Creating manager \n";
	bidsRecieved = 0;
	haveNoBids = true; //false;
	tasksRecieved = false;
	allBidsRecieved = false;
	//allComplete = false;
	ROBOT_ID = -1;
	for (int i = 0; i < 11; i++) {
		std::cout << "Creating channels \n";
		robotChanells.emplace_back(3 + i);
		//if (ROBOT_ID%2 == 1) robotChanells[i] = (ROBOT_ID + i)%10 + 3;
		//else robotChanells[i] = (ROBOT_ID-i>0) ? (ROBOT_ID - i) + 1 + 3 : 10 - abs(ROBOT_ID - i) + 3;
	}
}

void CBAA_Manager::AuctionPhase(EPuck_R * mRobot)
{
	timestep = mRobot->GetRobot()->getBasicTimeStep();
	long currentTime = 0, deltaTime = 0, previousTime = 0;
	mRobot->ExecuteBehaviour(TypeDefinitions::BEHAVIOUR_SAFE_WAIT);
	float heuristic = 0;
	for (std::deque<int>::iterator taskId = taskIds.begin(); taskId < taskIds.end(); taskId++) {
		if ((*taskId) >= -1) {
			// This is shit unless its static
			//heuristic = (mRobot->TaskPriority(taskMap[(*taskId)]) > bids[(*taskId)]) ? mRobot->TaskPriority(taskMap[(*taskId)]) : 0;
			//This is better
			heuristic = mRobot->TaskPriority(taskMap[(*taskId)]);
			//std::cout << "R_ID: " << ROBOT_ID << "Value for task: " << heuristic << "\n";
			if (heuristic > 0) {
				//std::cout << "R_ID: " << ROBOT_ID << "Going to do task: " << (*taskId) << "\n";
				myList[(*taskId)] = true;
				bids[(*taskId)] = heuristic;
			}
			if (mRobot->GetRobot()->step(timestep) == -1) {
				std::cout << "Work done: \n";
				mRobot->EndOfService();
			}
			currentTime += timestep;
			deltaTime = currentTime - previousTime;
			previousTime = currentTime;
			//std::cout << "R_ID: " << ROBOT_ID << "Gonna update: \n";
			mRobot->Update(nullptr, deltaTime);
			//previousTime = UpdateRobot(mRobot, timestep, currentTime, previousTime);
		}
		else {
			std::cout << "Task id is negative - something went wrong\n";
		}
	}
}

void CBAA_Manager::PlaceBids(EPuck_R * mRobot)
{
	AuctionPhase(mRobot);
}

int CBAA_Manager::ParseBidsMessage(std::string * buffer)
{
	//std::cout << "Got bids message " << *buffer << "\n";
	int bidderId = MessageTranslator::getRobotId(buffer);
	//std::cout << "Got ID " << bidderId << "\n";
	bidsRecieved = ((bidsRecieved) | (1 << bidderId));
	//std::cout << " Bids recieved: " << bidsRecieved << "\n";
	bidListsRecieved.emplace(bidderId, *buffer);
	//std::cout << "Emplaced \n";
	int subStart = buffer->find_first_of(':', buffer->find_first_of('-')) + 1;
	int subEnd = buffer->find_first_of(':', subStart);
	//std::cout << "Found indexes\n";
	std::string substring = "";
	do {
		if (subEnd != std::string::npos)substring.append(buffer->substr(subStart, subEnd - subStart));
		else substring.append(buffer->substr(subStart));
		//std::cout << "Current substring: " <<substring << "\n";
		//std::cout << "Task ID: " << substring.substr(0, substring.find_first_of(' ')) << "\n";
		//std::cout << "Bid: " << substring.substr(substring.find_first_of(' ') + 1) << "\n";
		bidsRec.emplace(stoi(substring.substr(0, substring.find_first_of(' '))), stof(substring.substr(substring.find_first_of(' ') + 1)));
		if (subEnd != std::string::npos) {
			subStart = subEnd + 1; //+ 1;
			subEnd = buffer->find_first_of(':', subStart);
		}
		substring.erase();
	} while (subEnd != std::string::npos);
	substring.append(buffer->substr(subStart));
	bidsRec.emplace(stoi(substring.substr(0, substring.find_first_of(' '))), stof(substring.substr(substring.find_first_of(' ') + 1)));
	substring.erase();
	return bidderId;
}

void CBAA_Manager::ParseBids()
{
	bidsMessage.append("B: ");
	bidsMessage.append(std::to_string(ROBOT_ID));
	bidsMessage.append(" - ");
	if (!bids.empty()) {
		for (auto it = bids.begin(); it != bids.end(); it++) {
			bidsMessage.append(":");
			bidsMessage.append(std::to_string((*it).first));
			bidsMessage.append(" ");
			bidsMessage.append(std::to_string((*it).second));
		}
		bidsMessage.append("\0");
		//std::cout << "R_ID: " << ROBOT_ID << "BIDS to send: " << bidsMessage << "\n";
	}
}

bool CBAA_Manager::WaitToRecieveBids(EPuck_R * mRobot, long TIME_OUT)
{
	long currentTime = 0, deltaTime = 0, previousTime = 0;
	long startTime = mRobot->GetTimeCounter(), timePassed = 0, i = 0;
	while (mRobot->GetCommunicationDevice()->GetNumberOfPackets() <= 0 && timePassed < TIMED_OUT) { // add safety for infinite loop
		timePassed = (mRobot->GetTimeCounter() - startTime);
		i++;
		//std::cout << "Started: " << startTime << " Current time: " << mRobot->GetTimeCounter() << "Timestep: "<< timestep << " Test iterator i:"<< i << "\n";
		//std::cout << "R_ID: " << ROBOT_ID << " waiting for "<< timePassed <<"\n";
		currentTime = UpdateRobot(mRobot, timestep, currentTime, previousTime);
		previousTime = currentTime;
	}
	std::cout << "R_ID: " << ROBOT_ID << " waited for " << timePassed << "\n";
	return (timePassed < TIMED_OUT);
}

bool CBAA_Manager::ConsensusPhase(EPuck_R * mRobot)
{
	long currentTime = 0, deltaTime = 0, previousTime = 0;
	long startTime = mRobot->GetTimeCounter(), timePassed = 0;
	int i = 0;
	// Consensus phase
	while (!allBidsRecieved && timePassed < (TIMED_OUT * 1000)) {
		timePassed = mRobot->GetTimeCounter() - startTime;
		ParseBids();
		if (!bids.empty() && bidsRecieved == 0) {
			// add condition to limit sending
			mRobot->GetCommunicationDevice()->SendMessage(bidsMessage.c_str(), bidsMessage.length());
			bidsRecieved = ((bidsRecieved) | (1 << ROBOT_ID));
		}
		else if (!bids.empty()){
			std::cout << " Bids already sent\n";
		}
		else {
			std::cout << " No auction yet\n";
			bidsRecieved = 0;
			return false;
		}
		bidsMessage.erase();
		//std::cout << " Bids sent\n";
		// Waiting for bids to arrive
		
		if (!WaitToRecieveBids(mRobot, TIMED_OUT)) {
			std::cout << "Timed out waiting for bids " << i++ << " times; Waiting for " << bidsRecieved << " to send bids ; in ConsensusPhase \n";
			bidsRecieved = 0; // rethink
		}
		// Process Bids
		//std::cout << " Got packet or timed out\n";
		while ((mRobot->GetCommunicationDevice()->GetNumberOfPackets() > 0)) {
			//std::cout << " Got packet in Consensus phase\n";
			ParsePacket(mRobot);
			currentTime = previousTime = UpdateRobot(mRobot, timestep, currentTime, previousTime);
			if (tasksRecieved) {
				//std::cout << " Got new task during Consensus phase\n";
				bidsRecieved = 0;
				tasksRecieved = false;
				return false;
			}
		}
		allBidsRecieved = (bidsRecieved == 0b11111111110);
		// Update
		currentTime = previousTime = UpdateRobot(mRobot, timestep, currentTime, previousTime);
		if (tasksRecieved) {
			bidsRecieved = 0;
			tasksRecieved = false;
			return false;
		}
	}

	if (!allBidsRecieved) {
		for (const auto& ti : taskIds) {
			if (timePassed >= (TIMED_OUT * 200) && ti != ROBOT_ID) {
				myList[ti] = false;
			}
		}
	}
	bidsRecieved = 0;
	allBidsRecieved = false;
	return true;
}

void CBAA_Manager::NegotiateConsensus(int negotiatorId)
{
	bool isOutbid = false;
	bool isEqual = false;
	//std::cout << "R_ID: " << ROBOT_ID << " Negotiating consensus: \n";
	for (std::deque<int>::iterator taskId = taskIds.begin(); taskId < taskIds.end(); taskId++) {
		//isOutbid = false;
		//std::cout << "T_ID: " << *taskId << " my bid: " << bids[(*taskId)]<< " negotiator bid: " << bidsRec[(*taskId)] << " Diff: " << bids[(*taskId)] - bidsRec[(*taskId)] << "\n";
		isOutbid = (bids[(*taskId)] < bidsRec[(*taskId)]);
		isEqual = (bids[(*taskId)] == bidsRec[(*taskId)]);
		
		//bids[(*taskId)] = fmax(bids[(*taskId)], bidsRec[(*taskId)]);
		if (isOutbid || (isEqual && (ROBOT_ID > negotiatorId))) {
			myList[(*taskId)] = false;
		}
	}
}

long CBAA_Manager::UpdateRobot(EPuck_R * mRobot, int timestep, long currentTime, long previousTime)
{
	if (mRobot->GetRobot()->step(timestep) == -1) {
		std::cout << "R_ID: " << ROBOT_ID << "Work done: \n";
		mRobot->EndOfService();
	}
	currentTime += timestep;
	long deltaTime = currentTime - previousTime;
	//std::cout << "R_ID: " << ROBOT_ID << " dt : " << deltaTime << "\n";
	//std::cout << "R_ID: " << ROBOT_ID << " ct : " << currentTime << "\n";
	//std::cout << "R_ID: " << ROBOT_ID << " pt : " << previousTime << "\n";
	previousTime = currentTime;
	//std::cout << "R_ID: " << ROBOT_ID << "Gonna update w dt : "<< deltaTime<<"\n";
	mRobot->Update(nullptr, deltaTime);
	//std::cout << "R_ID: " << ROBOT_ID << "Updated: \n";
	return previousTime;
}

bool CBAA_Manager::ParsePacket(EPuck_R * mRobot)
{
	buffer.append(mRobot->GetCommunicationDevice()->ReturnLastPacket()->c_str());
	int id = mRobot->OnMessageRecievedCBAA(&buffer, &taskMap); // creates task, if one is recieved and returns task id
	if (id >= 0) {
		std::cout << "R_ID: #" << ROBOT_ID << " Task recieved: " << id <<" in ParsePacket() \n";
		// This is done in mRobot 
		//taskMap.emplace(id, mRobot->GetTask(id));
		//std::cout << "ID: " << id << "\n";
		taskIds.emplace_back(id);
		//std::cout << "Bid: " << 0 << "\n";
		bids.emplace(id, 0);
		//std::cout << "Is it my task: " << false << "\n";
		myList.emplace(id, false);
		tasksRecieved = true;
		//std::cout << "Finished creating task \n";
	}
	else if (id == -3) {
		//std::cout << "Parsing bids " << buffer << " \n";
		int bidderId = ParseBidsMessage(&buffer);
		//Consensus
		NegotiateConsensus(bidderId);
		bidsRec.clear();
		tasksRecieved = false;
	}
	else if (id = -4) {
		std::cout << "R_ID: #" << ROBOT_ID << " Task completed by another robot " << id << " in ParsePacket() \n";
		tasksRecieved = false;
	}
	buffer.erase();
	return tasksRecieved;
}

bool CBAA_Manager::AllComplete()
{
	for (const auto& tp : taskMap) {
		if (tp.second->GetState() != TypeDefinitions::TASK_COMPLETE) return false;
	}
	return true;
}

bool CBAA_Manager::TA_CBAA(EPuck_R * mRobot)
{
	bidsRecieved = 0;
	allBidsRecieved = false;
	while (mRobot->GetCommunicationDevice()->GetNumberOfPackets() > 0) {
		ParsePacket(mRobot);
		AuctionPhase(mRobot);
		if (ROBOT_ID == 1) printDebug(4);
		if (ConsensusPhase(mRobot)) {
			printDebug(6);
			return true;
		}
		else return false;
	}
}

void CBAA_Manager::SetRobotId(int id)
{
	ROBOT_ID = id;
}

void CBAA_Manager::ResetTasksRecieved()
{
	tasksRecieved = false;
}

int CBAA_Manager::GetRobotId()
{
	return ROBOT_ID;
}

std::map<int, Task*> * CBAA_Manager::GetTaskMap()
{
	return &taskMap;
}

std::deque<int>* CBAA_Manager::GetTaskIds()
{
	return &taskIds;
}

std::map<int, float>* CBAA_Manager::GetBids()
{
	return &bids;
}

std::map<int, bool>* CBAA_Manager::GetMyList()
{
	return &myList;
}

void CBAA_Manager::printDebug(int debugType)
{
	switch (debugType)
	{
	case 1: // Print contents of all collections
		std::cout << "Task ids: \n";
		for (const auto& ti : taskIds) {
			std::cout << "Task id: " << ti << "\n";
		}
		std::cout << "\n Tasks: \n";
		for (const auto& tp : taskMap) {
			std::cout << "Task id: " << tp.first << " Task: " << tp.second->toString()->c_str() << "\n";
		}
		std::cout << "\n Bids: \n";
		for (const auto& b : bids) {
			std::cout << "Task id: " << b.first << " Bid: " << b.second << "\n";
		}
		std::cout << "\n MyList: \n";
		for (const auto& yes : myList) {
			std::cout << "Task id: " << yes.first << " Do I do: " << yes.second << "\n";
		}
		break;
	case 2: // Message by task
		for (const auto& ti : taskIds) {
			std::cout << "Task id: " << ti;
			std::cout << "\t Task: " <<taskMap.at(ti);
			std::cout << "\t Bid: " << bids.at(ti);
			std::cout << "\t Assigned to me: " << myList.at(ti) << "\n";
		}
		break;
	case 3: // print tabbed myList
		std::cout << "My doable tasks #"<< ROBOT_ID << ": ";
		for (const auto& ti : taskIds) {
			std::cout << "\t " << myList.at(ti);
		}
		std::cout << "\n";
		break;
	case 4: // Print tabbed ids
		std::cout << "My doable tasks #" << ROBOT_ID << ": ";
		for (const auto& ti : taskIds) {
			std::cout << "\t " << ti;
		}
		std::cout << "\n";
		break;
	case 5: // Print doable, but mark completed as 2
		std::cout << "My doable tasks #" << ROBOT_ID << ": ";
		for (const auto& ti : taskIds) {
			if(taskMap.at(ti)->GetState() == TypeDefinitions::TASK_COMPLETE) std::cout << "\t 2";
			else std::cout << "\t " << myList.at(ti);
		}
		std::cout << "\n";
		break;
	case 6: // Print doable, but mark completed as 2
		std::cout << "My doable tasks #" << ROBOT_ID << ": ";
		for (const auto& ti : taskIds) {
			if (taskMap.at(ti)->GetState() == TypeDefinitions::TASK_COMPLETE) std::cout << "\t 2";
			else std::cout << "\t " << myList.at(ti) << ":" << bids.at(ti);
		}
		std::cout << "\n";
		break;
	default:
		break;
	}
}

CBAA_Manager::~CBAA_Manager()
{
}
