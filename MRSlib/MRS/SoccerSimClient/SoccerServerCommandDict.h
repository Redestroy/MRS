#pragma once
#include <map>
#include <fstream>
#include <string>
#include <cstring>
#include <deque>
#include <iostream>
#include <MRS/Include/DistanceView.hpp>
namespace MRS {
	class SoccerServerCommandDict
	{
	private:
		std::map < std::string, std::pair<std::string, std::string>> command_dict;
		//std::map < std::string, std::string>> response_dict;
	public:
		SoccerServerCommandDict();
		~SoccerServerCommandDict(); 
		void ReadDictionary(std::string fileName);
		std::pair<std::string, std::string> GetCommand(std::string);
		std::string GetCommandTemplate(std::string);
		std::string GetArgsTemplate(std::string);

		static std::pair<std::string, std::string> ParseComPair(std::string* data);
		static std::string ParseArgsIntoTemplate(std::string data, std::deque<std::string> args);
		static std::map<std::string, std::string> ParseResponse_Init(std::string response);
		static std::map<std::string, std::string> ParseResponse_ServerParameters(std::string response);
		static std::map<std::string, std::string> ParseResponse_PlayerParameters(std::string response);
		static std::map<std::string, std::string> ParseResponse_PlayerTypeParameters(std::string response);
		static int ParsePlayerId(std::string response);
		static std::string ParsePacketId(std::string response);
		static std::map<std::string, std::string> ParseResponse_Hear(std::string response);
		static double ParseTimestamp(std::string response);
		static std::string ParseParameter(std::string parameter, std::string payload);
		static std::string ParseObjectParameter(std::string parameter, std::string payload);
		static size_t FindEndOfNextOpeningParentesies(std::string parameter, std::string parentesies, size_t start_position = 0);
		static Device::DistanceView ToPolRelCoords(std::string);
	};
}

