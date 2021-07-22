#include <cstdio>
#include <boost/asio.hpp>
#include <cstdlib>
#include <cstring>
#include <string>
#include <iostream>
#include <list>
#include <deque>
#include <map>
#include <fstream>
#include <rcsc/version.h>
#include <MRS/Include/Device.hpp>
#include <boost/timer/timer.hpp>
#include "UDPController.h"
#include "SoccerPlayerRobot.h"
#include "SpatialObjectSensor_UDPController.h"
#include "AudialSensor_UDPController.h"
#include "BodyInformationSensor_UDPController.h"
#include "MovementActuator2D_UDPController.h"
#include "BallActuator_UDPController.h"
#include "SoccerServerCommandDict.h"
#include "VoiceEmitter.h"

//#include <MRSbase/...> // Include for MRS linux library

std::pair<std::string, std::string> ParseComPair(std::string* data);
std::string ParseArgsIntoString(std::string* templ, std::string* arg);

using namespace boost::asio;
//using ip::udp;

// TODO after done:
// 1. Create UDPCommandDevice that contains the command dictionary with the command struct and all the methods shared by the devices
// 2. Change superclass from Robot to SoccerRobot - SoccerRobot will have general soccer related stuff added to it plus maybe an imbedded logic underneath

enum { max_length = 1024 };
int main(int argc, char* argv[])
{
    printf("librcsc Version %s\n", rcsc::version());
    try
    {
        if (argc != 4 && argc != 5)
        {
            std::cerr << "Usage: blocking_udp_echo_client <host> <port> <team> <goalie>\n";
            printf("Arg echo: arg1 : %s; \n \t arg2: %s \n \t arg3: %s\n \t arg4: %s\n", argv[0], argv[1], argv[2]);
            return 1;
        }
        std::cout << "In args 0\n";
        std::string apname = argv[0];
        std::cout << "In args 1\n";
        std::string IP_ADDRESS_str = argv[1];
        std::cout << "In args 2\n";
        unsigned short port = (unsigned short)std::stoi(argv[2]);
        std::cout << "In args 3\n";
        std::string team = argv[3];
        std::cout << "In args 4\n";
        std::string goalie = "";
        if(argc == 5)std::string goalie = argv[4];

        //read strings from file to populate command dictionary
        /*
        std::string command_string;
        std::list<std::string> command_strings;
        std::map<std::string, std::string> commands;
        std::deque<std::string> identifiers;
        std::fstream command_file;
        std::string INPUT_FILE_NAME = "CommandDict.dict";
        std::string INPUT_FILE_PATH = "//home//research//projects//SoccerSimClient//";
        std::string INPUT_FILE = INPUT_FILE_PATH + INPUT_FILE_NAME;
        command_file.open(INPUT_FILE, std::ios::in);
        if (command_file.is_open()) {
            while (std::getline(command_file, command_string)) {
                if (command_file.eof())break;
                //taskTimeline[i] = SECONDS_TO_STEPS * stoi(taskString.substr(0, taskString.find_first_of(' ')));
                command_strings.emplace_back(command_string);
                //std::cout << command_strings.back() << "\n";
            }
        }
        command_file.close();
        //end read strings from file

        std::string substr1, substr2, reply;
        //Command dict generation
        for (auto cmd_str : command_strings) {
            //std::cout << "Command first char: " << cmd_str[0] << "\n";
            if (cmd_str[0] != '#') {
                std::pair<std::string, std::string> cmd_pair = ParseComPair(&cmd_str);
                commands.emplace(cmd_pair.first, cmd_pair.second);
                identifiers.push_back(cmd_pair.first);
            }
        }
        */

        boost::timer::cpu_timer my_timer;

        MRS::SoccerServerCommandDict cmd_dict;
        cmd_dict.ReadDictionary("CommandDict.dict");
        std::cout << "Read dict\n";
        MRS::SoccerPlayerRobot* player;
        // create Agent
        if (argc == 4) {
            std::cout << "3 args\n";
            player = new MRS::SoccerPlayerRobot(IP_ADDRESS_str, port, team);
        }
        else if (argc == 5) {
            std::cout << "4 args\n";
            player = new MRS::SoccerPlayerRobot(IP_ADDRESS_str, port, team, goalie);
        }
        else {
            std::cout << "Improper args\n";
            return 2;
        }
        player->SetCommandDict(&cmd_dict);
        std::cout << "Created player\n";
        //MRS::Device::DeviceTree * devices = new MRS::Device::DeviceTree();
        MRS::Device::SensorBlock sensorBlock;
        MRS::Device::ActuatorBlock actuatorBlock;
            //create sensors
        MRS::Device::SpatialObjectSensor_UDPController* visual_sensor = new MRS::Device::SpatialObjectSensor_UDPController();
        //MRS::Device::AudialSensor_UDPController* audial_sensor = new MRS::Device::AudialSensor_UDPController();
        //MRS::Device::BodyInformationSensor_UDPController* body_sensor = new MRS::Device::BodyInformationSensor_UDPController();
        MRS::Device::MovementActuator2D_UDPController* move_actuator = new MRS::Device::MovementActuator2D_UDPController();
        MRS::Device::BallActuator_UDPController* ball_actuator = new MRS::Device::BallActuator_UDPController();
        MRS::Device::VoiceEmitter_UDPController* voice_box = new MRS::Device::VoiceEmitter_UDPController();
        // ComDev_UDP_EM voice_emitter = new Emitter
        // ComDev_UDP_Rec voice_receiver = audialSensor.GetReceiver()

        std::cout << "Created devices\n";

        sensorBlock.Add(((MRS::Device::SpatialObjectSensor*)visual_sensor));
        sensorBlock.Add(((MRS::Device::UDPSensor*)visual_sensor));
        std::cout << "Added sensors\n";
        //sensorBlock.Add(audial_sensor);
        //sensorBlock.Add(body_sensor);
        actuatorBlock.Add(move_actuator);
        move_actuator->SetCommandDict(&cmd_dict);
        move_actuator->SetUDPController(player->GetUDPController());

        voice_box->SetCommandDict(&cmd_dict);
        voice_box->SetUDPController(player->GetUDPController());
        player->SetCommunicationDevice(voice_box);
        //actuatorBlock.Add(ball_actuator);
        player->SetSensorBlock(&sensorBlock);
        player->SetActuatorBlock(&actuatorBlock);
        std::cout << "Added blocks to player\n";
        player->Init();
        std::cout << "Initialized player\n";
        // Run agent loop
        my_timer.start();
        auto time_start = my_timer.elapsed();
        boost::timer::nanosecond_type time_one = time_start.system, time_two = 0;
        long long delta = 0;
        double x1 = -50;
        double y1 = -30;
        while (true) {
#ifdef DEBUG
            //Debug for tasks and behaviours
            auto time_end = my_timer.elapsed();
            time_two = time_end.system;
            delta = time_two - time_one;
            if (delta > 1 * 100000000) {
                
            }
#else       
            auto time_end = my_timer.elapsed();
            time_two = time_end.system;
            delta = time_two - time_one;
            if (delta > 1 * 100000000) {
                std::cout << "Time elapsed " << delta <<"\n";
                time_start = my_timer.elapsed();
                time_one = time_start.system;
                player->Update(delta);
                //std::cout << "Move actuator " <<  << "\n";
                if (x1 < 50) {
                    x1 += 5;
                }
                else x1 = -50;
                if (y1 < 32) {
                    y1 += 5;
                }
                else y1 = -30;
                move_actuator->Move(x1, y1);
                //move_actuator->Dash(0.5, 0);
                //stuff happening once per tick

            }
            //if (player.GetGameState() != GOING) {
                //if not at start pos, move to start pos
                //if (player.pos != player.start_pos) player.move_to(player.start_pos);
                // else do nothing
           // }else{
            //      1st get sensor info
               // player->Update(1);
            //          send sensor packets
            //          parse sensor info
            //      2nd do logic
                //player->RunAlgorithm
                //do logic here with player and roles
            //          agent logic loop
            //      3rd communication
            //          send say command
            //          parse hear command`
            //      4th do action
                //player->Do();
            //          send actuator packets
            //          check replies
            //
            //}
#endif
        }
        
        printf("Arg echo: arg1 : %s; \n \t arg2: %s \n \t arg3: %s\n \t arg4: %s\n", argv[0], argv[1], argv[2], argv[3]);
        // Parse parameters
        

 
        /* print statements
        for (auto cmd : commands) {
            std::cout << "Command " << cmd.first << " : " << cmd.second << "\n";
        }
        std::cout << "Identifiers \n";
        for (auto cmd : identifiers) {
            std::cout << cmd << "\n";
        }

        for (auto cmd : commands) {
            substr1 = cmd.second.substr(0, cmd.second.find(';'));
            substr2 = cmd.second.substr(cmd.second.find(';')+1, cmd.second.find('\r')- cmd.second.find(';')-1);
            std::cout << "Command initialized " << cmd.first << " : " << ParseArgsIntoString(&substr1,&substr2) << "\n";
        }
        */
    }
    catch (std::exception & e)
    {
        std::cerr << "Exception: " << e.what() << "\n";
    }
    return 0;
}

std::pair<std::string, std::string> ParseComPair(std::string * data) {
    std::pair<std::string, std::string> com_pair;
    int currentParentesiesOpen = 0, next = 0, firstTemp, secondTemp;
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

std::string ParseArgsIntoString(std::string* templ, std::string* arg)
{
    // count necessary args 
    int ARG_COUNT_TEMPLATE = 0;
    int last_ArgInd = 0;
    while (templ->find("Arg_", last_ArgInd) != std::string::npos) {
        last_ArgInd = templ->find("Arg_", last_ArgInd);
        if ((*templ)[last_ArgInd-1] != '[') ARG_COUNT_TEMPLATE++;
        last_ArgInd += std::strlen("Arg_");
      //  std::cout << "LastArg " << last_ArgInd << "\n";
      //  std::cout << "LastArg2 " << templ->find("Arg_", last_ArgInd) << "\n";
    }
    // count given values
    int ARG_COUNT_VALUES = 0;
    last_ArgInd = 0;
    while (arg->find("=", last_ArgInd) != std::string::npos) {
        last_ArgInd = arg->find("=", last_ArgInd);
        last_ArgInd++;
        ARG_COUNT_VALUES++;
    }
   // std::cout << "For string <" << *templ << "> with values <" << *arg << ">\n";
   // std::cout << "Arg count: " << ARG_COUNT_TEMPLATE << " VS " << ARG_COUNT_VALUES << "\n";
    if(ARG_COUNT_TEMPLATE>ARG_COUNT_VALUES) return "";
    std::string result = "" + *templ;
    if (ARG_COUNT_TEMPLATE == 0 && ARG_COUNT_VALUES == 0) return result;
    std::string substr = "";
    int arg_num = 0;
    int replaced = 0;
    last_ArgInd = arg->find('=');
    int last_ArgEndInd = arg->find(' ', last_ArgInd+1);
    while (result.find("Arg_") != std::string::npos) {
        substr = arg->substr(last_ArgInd+1,last_ArgEndInd-last_ArgInd-1);
        last_ArgInd = arg->find('=', last_ArgEndInd+1);
        last_ArgEndInd = arg->find_first_of(" \r\n", last_ArgInd + 1);
        if (replaced < ARG_COUNT_VALUES) {
            result.replace(result.find("Arg_"), result.find_first_of(" )", result.find("Arg_") + 1) - result.find("Arg_"), substr);
            replaced++;
        }
        else {
            // replace [] with empty
            result.replace(result.find("[Arg_")-1, result.find_first_of(" ])", result.find("[Arg_") + 1) - result.find("[Arg_")+1, "");
        }
        //std::cout << "Result: " << result << "\n";
    }
    while (result.find_first_of("[]") != std::string::npos) {
        result.replace(result.find_first_of("[]"),1,"");
    }
    while (result.find_first_of("\"") != std::string::npos) {
        result.replace(result.find_first_of("\""), 1, "");
    }
    return result;
}
