#include "SoccerServerCommandDict.h"
namespace MRS {
	SoccerServerCommandDict::SoccerServerCommandDict()
	{
	}

	SoccerServerCommandDict::~SoccerServerCommandDict()
	{
	}

	void SoccerServerCommandDict::ReadDictionary(std::string fileName)
	{
        std::string command_string,substr1,substr2;
        std::map<std::string, std::string> commands;
        std::fstream command_file;
        //std::string INPUT_FILE_NAME = "CommandDict.dict";
        std::string INPUT_FILE_PATH = "//home//research//projects//SoccerSimClient//";
        std::string INPUT_FILE = INPUT_FILE_PATH + fileName;
        command_file.open(INPUT_FILE, std::ios::in);
        if (command_file.is_open()) {
            while (std::getline(command_file, command_string)) {
                if (command_file.eof())break;
                std::cout << "Start read\n";
                if (command_string[0] != '#') {
                    std::cout << "In read\n";
                    std::pair<std::string, std::string> cmd_pair = ParseComPair(&command_string);
                    std::cout << "In read 2\n";
                    substr1 = cmd_pair.second.substr(0, cmd_pair.second.find(';'));
                    substr2 = cmd_pair.second.substr(cmd_pair.second.find(';') + 1, cmd_pair.second.find('\r') - cmd_pair.second.find(';') - 1);
                    std::cout << "In read 3\n";
                    //std::cout << "Command initialized " << cmd.first << " : " << ParseArgsIntoString(&substr1, &substr2) << "\n";
                    std::pair<std::string, std::string> sub_pair = std::make_pair(substr1, substr2);
                    std::cout << "In read 4\n";
                    command_dict.emplace(cmd_pair.first, sub_pair);
                }
            }
        }
        command_file.close();
        //end read strings from file
	}

	std::pair<std::string, std::string> SoccerServerCommandDict::GetCommand(std::string key)
	{
		return command_dict[key];
	}
	std::string SoccerServerCommandDict::GetCommandTemplate(std::string key)
	{
		return command_dict[key].first;
	}
	std::string SoccerServerCommandDict::GetArgsTemplate(std::string key)
	{
		return command_dict[key].second;
	}

	std::pair<std::string, std::string> SoccerServerCommandDict::ParseComPair(std::string* data)
	{
        std::pair<std::string, std::string> com_pair;
        int currentParentesiesOpen = 0;
        std::string temp, payloadSubstring, identitySubstring;
        // copy
        temp = "" + *data;
        // get substring
        currentParentesiesOpen = (int)temp.find_first_of(':', 0);
        identitySubstring = "" + temp.substr(0, currentParentesiesOpen - 1);
        payloadSubstring = "" + temp.substr(currentParentesiesOpen + 2);
        com_pair.first = "" + identitySubstring;
        com_pair.second = "" + payloadSubstring;
        return com_pair;
	}

    std::string SoccerServerCommandDict::ParseArgsIntoTemplate(std::string data, std::deque<std::string> args) // this parses an arg list into the arg template
    {
        std::string result = "" + data;
        int ARG_COUNT_VALUES = (int)args.size();
        int ARG_COUNT_TEMPLATE = 0;

        int last_ArgInd = 0;
        while (data.find("=", last_ArgInd) != std::string::npos) {
            last_ArgInd = (int)data.find("=", last_ArgInd);
            ARG_COUNT_TEMPLATE++;
            last_ArgInd += (int)std::strlen("=");
        }
        std::cout << "Arg count values: " << ARG_COUNT_VALUES << "\n";
        std::cout << "Arg template values: " << ARG_COUNT_TEMPLATE << "\n";
        int MIN_COUNT = 0;
        if (ARG_COUNT_TEMPLATE < ARG_COUNT_VALUES) MIN_COUNT = ARG_COUNT_TEMPLATE;
        else MIN_COUNT = ARG_COUNT_VALUES;
        if (MIN_COUNT <= 0) return "";



        last_ArgInd = (int)result.find("=");
        int last_charInd = (int)result.find(" ");
        for (int i = 0; i < MIN_COUNT; i++) {
            /*
            switch (result[last_ArgInd + 1]) {
            case 'i':
                //int
                break;
            case 'd':
                //double
                return ARG_TYPE::DOUBLE;
            case 'b':
                //bool
                return ARG_TYPE::BOOLEAN;
            case 'c':
                //char
                return ARG_TYPE::CHARACTER;
            case 's':
                //string
                return ARG_TYPE::C_STRING;
            case 'o':
                //object
                break;
            case 'p':
                //pointer
                return ARG_TYPE::POINTER;
            default:
                return ARG_TYPE::CHARACTER;
            }
        */
            last_charInd = result.find_first_of(" )\r\n", last_ArgInd + 1);
            result.replace(last_ArgInd+1, result.find_first_of(" )\r\n", last_ArgInd+1)-last_ArgInd-1, args[i]);
            last_ArgInd = result.find("=", last_ArgInd +1);
            std::cout << "Arg string in "<< i <<" cycle: " << result << "\n";
        }
        if(ARG_COUNT_TEMPLATE != ARG_COUNT_VALUES) result.replace(last_charInd, result.size() - last_charInd - 1, ""); //TODO: check math
        std::cout << "Arg string: "<< result << "\n";
        return result;
    }

    std::map<std::string,std::string> SoccerServerCommandDict::ParseResponse_Init(std::string response)
    {
        std::map<std::string, std::string> response_map;
        double x = 0.0;
        int currentParentesiesOpen = 0, next = 0, firstTemp, secondTemp;
        std::string temp, dataSubstring, xSubstring;
        // copy
        temp = "" + response;
        // get substring
        currentParentesiesOpen = (int)temp.find_first_of('(', 0);
        next = (int)temp.find_first_of(' ', currentParentesiesOpen + 1);
        //this is init
        currentParentesiesOpen = next;
        next = (int)temp.find_first_of(' ', currentParentesiesOpen + 1);
        //this is side
        dataSubstring = "" + temp.substr(currentParentesiesOpen + 1, next - currentParentesiesOpen);
        response_map.emplace("side", dataSubstring);

        currentParentesiesOpen = next;
        next = (int)temp.find_first_of(' ', currentParentesiesOpen + 1);
        //this is id
        dataSubstring = "" + temp.substr(currentParentesiesOpen + 1, next - currentParentesiesOpen);
        response_map.emplace("id", dataSubstring);

        currentParentesiesOpen = next;
        next = (int)temp.find_first_of(')', currentParentesiesOpen + 1);
        //this is game phase
        dataSubstring = "" + temp.substr(currentParentesiesOpen + 1, next - currentParentesiesOpen);
        response_map.emplace("game_phase", dataSubstring);

        return response_map;
    }
    std::map<std::string, std::string> SoccerServerCommandDict::ParseResponse_ServerParameters(std::string response)
    {
        std::map<std::string, std::string> response_map;
        std::size_t currentParentesiesOpen = 0, next = 0, firstTemp, secondTemp;
        std::string temp, dataSubstring, xSubstring, firstSubstring, secondSubstring;
        // copy
        temp = "" + response;
        currentParentesiesOpen = (int)temp.find_first_of('(', 0);
        next = (int)temp.find_first_of(' ', currentParentesiesOpen + 1);
        // this is server_param
        currentParentesiesOpen = (int)temp.find_first_of('(', next);
        next = (int)temp.find_first_of(')', currentParentesiesOpen + 1);
        //
        std::cout << "Server parameters\n";
        std::cout << "-----------------\n";
        while (next != std::string::npos) {
            dataSubstring = temp.substr(currentParentesiesOpen + 1, next - currentParentesiesOpen);
            firstSubstring = dataSubstring.substr(0, dataSubstring.find_first_of(' '));
            secondSubstring = dataSubstring.substr(dataSubstring.find_first_of(' ') + 1);
            response_map.emplace(firstSubstring, secondSubstring);
            currentParentesiesOpen = (int)temp.find_first_of('(', next);
            next = (int)temp.find_first_of(')', currentParentesiesOpen + 1);
            std::cout << "Parameter: <" << dataSubstring << "> -> <" << firstSubstring << "> : <" << secondSubstring << ">\n";
        }
        return response_map;
    }
    std::map<std::string, std::string> SoccerServerCommandDict::ParseResponse_PlayerParameters(std::string response)
    {
        std::map<std::string, std::string> response_map;
        std::size_t currentParentesiesOpen = 0, next = 0, firstTemp, secondTemp;
        std::string temp, dataSubstring, xSubstring, firstSubstring, secondSubstring;
        // copy
        temp = "" + response;
        currentParentesiesOpen = (int)temp.find_first_of('(', 0);
        next = (int)temp.find_first_of(' ', currentParentesiesOpen + 1);
        // this is player_param
        currentParentesiesOpen = (int)temp.find_first_of('(', next);
        next = (int)temp.find_first_of(')', currentParentesiesOpen + 1);
        //
        std::cout << "Player parameters\n";
        std::cout << "-----------------\n";
        while (next != std::string::npos) {
            dataSubstring = temp.substr(currentParentesiesOpen + 1, next - currentParentesiesOpen);
            firstSubstring = dataSubstring.substr(0, dataSubstring.find_first_of(' '));
            secondSubstring = dataSubstring.substr(dataSubstring.find_first_of(' ') + 1);
            response_map.emplace(firstSubstring, secondSubstring);
            currentParentesiesOpen = (int)temp.find_first_of('(', next);
            next = (int)temp.find_first_of(')', currentParentesiesOpen + 1);
            std::cout << "Parameter: <" << dataSubstring << "> -> <" << firstSubstring << "> : <" << secondSubstring << ">\n";
        }
        return response_map;
    }
    std::map<std::string, std::string> SoccerServerCommandDict::ParseResponse_PlayerTypeParameters(std::string response)
    {
        std::map<std::string, std::string> response_map;
        std::size_t currentParentesiesOpen = 0, next = 0, firstTemp, secondTemp;
        std::string temp, dataSubstring, xSubstring, firstSubstring, secondSubstring;
        // copy
        temp = "" + response;
        currentParentesiesOpen = (int)temp.find_first_of('(', 0);
        next = (int)temp.find_first_of(' ', currentParentesiesOpen + 1);
        // this is player_param
        currentParentesiesOpen = (int)temp.find_first_of('(', next);
        next = (int)temp.find_first_of(')', currentParentesiesOpen + 1);
        dataSubstring = temp.substr(currentParentesiesOpen + 1, next - currentParentesiesOpen);
        // this is id
        firstSubstring = dataSubstring.substr(0, dataSubstring.find_first_of(' '));
        secondSubstring = dataSubstring.substr(dataSubstring.find_first_of(' ') + 1);
        response_map.emplace(firstSubstring, secondSubstring);
        std::cout << "Player type parameters\n";
        std::cout << "-----------------\n";
        std::cout << "ID: <" << dataSubstring << "> -> <" << firstSubstring << "> : <" << secondSubstring << ">\n";
        while (next != std::string::npos) {
            dataSubstring = temp.substr(currentParentesiesOpen + 1, next - currentParentesiesOpen);
            firstSubstring = dataSubstring.substr(0, dataSubstring.find_first_of(' '));
            secondSubstring = dataSubstring.substr(dataSubstring.find_first_of(' ') + 1);
            response_map.emplace(firstSubstring, secondSubstring);
            currentParentesiesOpen = (int)temp.find_first_of('(', next);
            next = (int)temp.find_first_of(')', currentParentesiesOpen + 1);
            std::cout << "Parameter: <" << dataSubstring << "> -> <" << firstSubstring << "> : <" << secondSubstring << ">\n";
        }
        return response_map;
    }
    int SoccerServerCommandDict::ParsePlayerId(std::string response)
    {
        std::size_t currentParentesiesOpen = 0, next = 0, firstTemp, secondTemp;
        std::string temp, dataSubstring, xSubstring, firstSubstring, secondSubstring;
        // copy
        temp = "" + response;
        currentParentesiesOpen = (int)temp.find_first_of('(', 0);
        next = (int)temp.find_first_of(' ', currentParentesiesOpen + 1);
        // this is player_param
        currentParentesiesOpen = (int)temp.find_first_of('(', next);
        next = (int)temp.find_first_of(')', currentParentesiesOpen + 1);
        dataSubstring = temp.substr(currentParentesiesOpen + 1, next - currentParentesiesOpen);
        // this is id
        secondSubstring = dataSubstring.substr(dataSubstring.find_first_of(' ') + 1);
        try {
            return std::stoi(secondSubstring);
        }
        catch (...) {
            return -1;
        }
    }
    std::string SoccerServerCommandDict::ParsePacketId(std::string response)
    {
        std::size_t currentParentesiesOpen = 0, next = 0;
        std::string temp, dataSubstring;
        // copy
        temp = "" + response;
        currentParentesiesOpen = (int)temp.find_first_of('(', 0);
        next = (int)temp.find_first_of(' ', currentParentesiesOpen + 1);
        // this is packet id
        dataSubstring = temp.substr(currentParentesiesOpen + 1, next - currentParentesiesOpen);
        return dataSubstring;
    }
    std::map<std::string, std::string> SoccerServerCommandDict::ParseResponse_Hear(std::string response)
    {
        std::map<std::string, std::string> response_map;
        double x = 0.0;
        int currentParentesiesOpen = 0, next = 0, firstTemp, secondTemp;
        std::string temp, dataSubstring, xSubstring;
        // copy
        temp = "" + response;
        // get substring
        currentParentesiesOpen = (int)temp.find_first_of('(', 0);
        next = (int)temp.find_first_of(' ', currentParentesiesOpen + 1);
        //this is hear
        currentParentesiesOpen = next;
        next = (int)temp.find_first_of(' ', currentParentesiesOpen + 1);
        //this is Time
        dataSubstring = "" + temp.substr(currentParentesiesOpen + 1, next - currentParentesiesOpen);
        response_map.emplace("time", dataSubstring);

        currentParentesiesOpen = next;
        next = (int)temp.find_first_of(' ', currentParentesiesOpen + 1);
        //this is Sender
        dataSubstring = "" + temp.substr(currentParentesiesOpen + 1, next - currentParentesiesOpen);
        response_map.emplace("sender", dataSubstring);

        currentParentesiesOpen = next;
        next = (int)temp.find_first_of("\"", currentParentesiesOpen + 1);
        currentParentesiesOpen = next;
        next = (int)temp.find_first_of("\"", currentParentesiesOpen + 1);
        //this is messege
        dataSubstring = "" + temp.substr(currentParentesiesOpen + 1, next - currentParentesiesOpen);
        response_map.emplace("payload", dataSubstring);

        return response_map;
    }

    double SoccerServerCommandDict::ParseTimestamp(std::string response)
    {
        std::map<std::string, std::string> response_map;
        double x = 0.0;
        int currentParentesiesOpen = 0, next = 0, firstTemp, secondTemp;
        std::string temp, dataSubstring, xSubstring;
        currentParentesiesOpen = (int)temp.find_first_of('(', 0);
        next = (int)temp.find_first_of(' ', currentParentesiesOpen + 1);
        //this is see or hear
        currentParentesiesOpen = next;
        next = (int)temp.find_first_of(' ', currentParentesiesOpen + 1);
        // this is timestamp
        dataSubstring = "" + temp.substr(currentParentesiesOpen + 1, next - currentParentesiesOpen);
        try {
            x = std::stod(dataSubstring);
        }
        catch (...) {
            std::cout << "Could not parse: " << dataSubstring << "\n";
            x = -3.14;
        }
        return x;
    }
    std::string SoccerServerCommandDict::ParseParameter(std::string parameter, std::string payload)
    {
        // search string for parameter
        // then crop next item
        // then return
        size_t position = payload.find(parameter);
        if (position == std::string::npos) return "";
        std::string response = "";
        size_t first_index = payload.find(" ", position + 1);
        size_t second_index = payload.find_first_of(" )", first_index+1);
        response = payload.substr(first_index+1, second_index - first_index);
        return response;
    }
    std::string SoccerServerCommandDict::ParseObjectParameter(std::string parameter, std::string payload)
    {
        // string is passed without outside parentesies
        int currentParentesiesOpen = 0, next = 0, firstTemp, secondTemp;
        std::string temp, dataSubstring, xSubstring;
        temp = "" + payload;
        currentParentesiesOpen = (int)temp.find_first_of('(', 0);
        next = FindEndOfNextOpeningParentesies(temp,"()");
        if (parameter.compare("ObjName") == 0) {
            dataSubstring = "" + temp.substr(currentParentesiesOpen + 1, next - currentParentesiesOpen);
            return dataSubstring;
        }
        currentParentesiesOpen = (int)temp.find_first_of(' ', next);
        next = (int)temp.find_first_of(' ', currentParentesiesOpen + 1);
        if (parameter.compare("Distance") == 0) {
            dataSubstring = "" + temp.substr(currentParentesiesOpen + 1, next - currentParentesiesOpen);
            return dataSubstring;
        }
        currentParentesiesOpen = next;
        next = (int)temp.find_first_of(' ', currentParentesiesOpen + 1);
        if (parameter.compare("Direction") == 0) {
            dataSubstring = "" + temp.substr(currentParentesiesOpen + 1, next - currentParentesiesOpen);
            return dataSubstring;
        }
        currentParentesiesOpen = next;
        next = (int)temp.find_first_of(' ', currentParentesiesOpen + 1);
        if (parameter.compare("DistChange") == 0) {
            dataSubstring = "" + temp.substr(currentParentesiesOpen + 1, next - currentParentesiesOpen);
            return dataSubstring;
        }
        currentParentesiesOpen = next;
        next = (int)temp.find_first_of(' ', currentParentesiesOpen + 1);
        if (parameter.compare("DirChange") == 0) {
            dataSubstring = "" + temp.substr(currentParentesiesOpen + 1, next - currentParentesiesOpen);
            return dataSubstring;
        }
        currentParentesiesOpen = next;
        next = (int)temp.find_first_of(' ', currentParentesiesOpen + 1);
        if (parameter.compare("BodyFacingDir") == 0) {
            dataSubstring = "" + temp.substr(currentParentesiesOpen + 1, next - currentParentesiesOpen);
            return dataSubstring;
        }
        currentParentesiesOpen = next;
        next = (int)temp.find_first_of(' ', currentParentesiesOpen + 1);
        if (parameter.compare("HeadFacingDir") == 0) {
            dataSubstring = "" + temp.substr(currentParentesiesOpen + 1, next - currentParentesiesOpen);
            return dataSubstring;
        }
        currentParentesiesOpen = next;
        next = (int)temp.find_first_of(' ', currentParentesiesOpen + 1);
        if (parameter.compare("PointDir") == 0) {
            dataSubstring = "" + temp.substr(currentParentesiesOpen + 1, next - currentParentesiesOpen);
            return dataSubstring;
        }
        currentParentesiesOpen = next;
        next = (int)temp.find_first_of(' ', currentParentesiesOpen + 1);
        if (parameter.compare("Tackling") == 0) {
            dataSubstring = "" + temp.substr(currentParentesiesOpen + 1, next - currentParentesiesOpen);
            return dataSubstring;
        }
        currentParentesiesOpen = next;
        next = (int)temp.find_first_of(' ', currentParentesiesOpen + 1);
        if (parameter.compare("Kicking") == 0) {
            dataSubstring = "" + temp.substr(currentParentesiesOpen + 1, next - currentParentesiesOpen);
            return dataSubstring;
        }

        if (parameter.compare("Coords") == 0) {
            xSubstring = "";
            xSubstring += ParseObjectParameter("Distance", payload);
            xSubstring += " ";
            xSubstring += ParseObjectParameter("Direction", payload);
            return xSubstring;
        }
        return "";
    }
	size_t SoccerServerCommandDict::FindEndOfNextOpeningParentesies(std::string text, std::string parentesies, size_t start_position)
	{
        // returns position of final parentesies, even if  there are parentesies in the middle
        // guarantied to find the other parentesies or return std::string::npos
        // not guarantied for all possible parentesies types
        // also
        int open_counter = 0;
        size_t current_pos = text.find(parentesies[0], start_position);
        if (current_pos != std::string::npos) {
            open_counter++;
            current_pos = text.find_first_of(parentesies, current_pos + 1);
            while (open_counter != 0 || current_pos != std::string::npos) {
                if (text[current_pos] == parentesies[0]) {
                    open_counter++;
                }
                else{
                    open_counter--;
                }
                current_pos = text.find_first_of(parentesies, current_pos + 1);
            }
            return current_pos;
        }
		return std::string::npos;
	}
    Device::DistanceView SoccerServerCommandDict::ToPolRelCoords(std::string rel_position)
    {
        double distance = -1;
        double direction = -1;
        Device::DistanceView pos;
        try {
            distance = std::stod(rel_position.substr(0, rel_position.find(" "))); // TODO: change space to seperator that is passed as default
            distance = std::stod(rel_position.substr(rel_position.find(" ")));
            pos.SetDistance(distance);
            pos.SetAngle(direction);
        }
        catch (...) {
            std::cerr << "Position parse failed on string: " << rel_position << "\n";
            pos.SetDistance(-1);
            pos.SetAngle(-1);
        }
        return pos;
    }
}
