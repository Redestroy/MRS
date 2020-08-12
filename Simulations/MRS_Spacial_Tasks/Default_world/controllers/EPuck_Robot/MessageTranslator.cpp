#include "MessageTranslator.h"



MessageTranslator::MessageTranslator()
{
}


MessageTranslator::~MessageTranslator()
{
}

bool MessageTranslator::isValid(std::string * packet, int packetSize)
{	
	if (packetSize >= 256) return false;
	//if (packet->find_first_of('\n') == packetSize - 1) {
		if (getMessageType((*packet)[0]) != TypeDefinitions::INVALID_MESSAGE) {
			if ((*packet)[1] == ':' && (*packet)[2] == ' ') return true;
		}
	//}
	return false;
}

bool MessageTranslator::isValidCBAA(std::string * packet, int packetSize)
{
	if (packetSize >= 256) return false;
	//if (packet->find_first_of('\n') == packetSize - 1) {
	if (getMessageTypeCBAA((*packet)[0]) != TypeDefinitions::CBAA_INVALID_MESSAGE) {
		if ((*packet)[1] == ':' && (*packet)[2] == ' ') return true;
	}
	//}
	return false;
}

TypeDefinitions::MessageType MessageTranslator::getMessageType(char buffer)
{
	switch (buffer)
	{
	case 'P':
		return TypeDefinitions::PING;
	case 'E':
		return TypeDefinitions::TASK_COMPLETE_MESSAGE;
	case 'T':
		return TypeDefinitions::TASK_MESSAGE;
	case 'G':
		return TypeDefinitions::GPS_REQUEST;
	default:
		return TypeDefinitions::INVALID_MESSAGE;
		break;
	}
}

TypeDefinitions::MessageType_CBAA MessageTranslator::getMessageTypeCBAA(char buffer)
{
	switch (buffer)
	{
	case 'P':
		return TypeDefinitions::CBAA_PING;
	case 'E':
		return TypeDefinitions::CBAA_TASK_COMPLETE_MESSAGE;
	case 'B':
		return TypeDefinitions::CBAA_BIDS_MESSAGE;
	case 'T':
		return TypeDefinitions::CBAA_TASK_MESSAGE;
	case 'G':
		return TypeDefinitions::CBAA_GPS_REQUEST;
	default:
		return TypeDefinitions::CBAA_INVALID_MESSAGE;
		break;
	}
}

int MessageTranslator::getTaskIndex(std::string * packet)
{
	switch (getMessageType((*packet)[0])) {
	case TypeDefinitions::TASK_COMPLETE_MESSAGE:
		return std::stoi(packet->substr(packet->find_first_of(' ')).c_str());
	case TypeDefinitions::TASK_MESSAGE:
		//std::cout << "Task id to check: " << std::stoi((*packet).substr(packet->find_first_of(' ') + 1,
		//	packet->find_first_of(' ', packet->find_first_of(' ') + 1) - (packet->find_first_of(' ') + 1))) << "\n";
		return std::stoi((*packet).substr(packet->find_first_of(' ') + 1,
			packet->find_first_of(' ', packet->find_first_of(' ') + 1)-(packet->find_first_of(' ') + 1)));
	default:
		return -1;
	}
	return -1;
}

int MessageTranslator::getRobotId(std::string * packet)
{
	switch (getMessageTypeCBAA((*packet)[0])) {
	case TypeDefinitions::TASK_COMPLETE_MESSAGE:
		return std::stoi(packet->substr(packet->find_first_of(' ')).c_str());
	case TypeDefinitions::TASK_MESSAGE:
		//std::cout << "Task id to check: " << std::stoi((*packet).substr(packet->find_first_of(' ') + 1,
//			packet->find_first_of(' ', packet->find_first_of(' ') + 1) - (packet->find_first_of(' ') + 1))) << "\n";
		return std::stoi((*packet).substr(packet->find_first_of(' ') + 1,
			packet->find_first_of(' ', packet->find_first_of(' ') + 1) - (packet->find_first_of(' ') + 1)));
	case TypeDefinitions::CBAA_BIDS_MESSAGE: 
		//std::cout << "Biding robot id: " << std::stoi((*packet).substr(packet->find_first_of(' ') + 1,
			//packet->find_first_of(' ', packet->find_first_of(' ') + 1) - (packet->find_first_of(' ') + 1))) << "\n";
		return std::stoi((*packet).substr(packet->find_first_of(' ') + 1,
			packet->find_first_of(' ', packet->find_first_of(' ') + 1) - (packet->find_first_of(' ') + 1)));
	default:
		return -1;
	}
	return -1;
}
