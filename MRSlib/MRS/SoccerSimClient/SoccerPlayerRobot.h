#pragma once
//#include <MRS/Include/Robot.hpp>
#include <MRS/Include/DeviceTree.hpp>
#include <MRS/Include/Device.hpp>
#include <MRS/Include/Sensor.hpp>
#include <MRS/Include/Actuator.hpp>
#include <MRS/Include/Action.hpp>
#include <MRS/Include/SensorBlock.hpp>
#include <MRS/Include/ActuatorBlock.hpp>
#include "UDPController.h"
#include "SoccerServerCommandDict.h"
#include "VoiceEmitter.h"
#include <rcsc/player/player_agent.h>
#include <string>
#include <deque>
//From agent2D
//TODO: move to emiter and other drivers
//#include "../chain_action/action_generator.h"
//#include "../chain_action/field_evaluator.h"
//#include "../communication.h"

#define RIGHT true
#define LEFT false

namespace MRS {
	enum class TEAM_ROLE
	{
		GOALIE,
		DEFENSE,
		OFFENSE,
		NOT_ASSIGNED
	};

	enum class GAME_PHASE
	{
		BEFORE_KICK_OFF,
		OTHER
	};

	class SoccerPlayerRobot : rcsc::PlayerAgent //: public MRS::Device::Robot
	{
	private:
		// Team info
		std::string team_name;
		int id;
		bool side; // l - false, r - true
		GAME_PHASE phase;
		//bool goalie
		std::deque<std::string> args;
		std::map<std::string, std::string> server_parameters, player_parameters;
		std::map<int, std::map< std::string, std::string>> player_type_parameters;
		TEAM_ROLE role;
		UDPController* udpController;
		Device::SensorBlock* sensors;
		Device::ActuatorBlock * actuators;
		Device::DeviceTree* devices;
		SoccerServerCommandDict* cmd_dict;
		Device::VoiceEmitter_UDPController* com_device;

		// Base agent stuff
		std::deque<Device::View*> * percepts; // used to invoke agent stuff in robot - gives percepts to Robot
		std::deque<Device::Action*>* actions; // used to invoke agent stuff in robot - stores actions for Robot to execute
		
		//rcsc::Agent prerequisites
		// copied from sample agent

		//Communication::Ptr M_communication;
		//FieldEvaluator::ConstPtr M_field_evaluator;
		//ActionGenerator::ConstPtr M_action_generator;

		virtual
			bool initImpl(rcsc::CmdLineParser& cmd_parser);

		//! main decision
		virtual
			void actionImpl();

		//! communication decision
		virtual
			void communicationImpl();

		virtual
			void handleActionStart();
		virtual
			void handleActionEnd();

		virtual
			void handleServerParam();
		virtual
			void handlePlayerParam();
		virtual
			void handlePlayerType();

		virtual
			FieldEvaluator::ConstPtr createFieldEvaluator() const;

		virtual
			ActionGenerator::ConstPtr createActionGenerator() const;

		//rcsc::Agent management
		

		// World

		// Behaviours / actions
		// basic
		// kick 
		// catch
		// run_to
		// intermediate
		// intercept - run_to(ball, fast)
		// score - kick(goal)
		// drible - kick(self+delta)
		//			run_to(delta, normal)
		// pass - kick(team-mate)
		// defend - run_to(mid(ball,goal),normal/fast)
		// standby_walk - run_to(random, slow)
		// advanced
		// switch
		// hudle - communication event between multiple agents
		// defend
		// 
		// Devices 
	public:
		SoccerPlayerRobot();
		SoccerPlayerRobot(std::string, unsigned short, std::string);
		SoccerPlayerRobot(std::string, unsigned short, std::string, bool);
		SoccerPlayerRobot(std::string, unsigned short, std::string, std::string);
		~SoccerPlayerRobot();
		virtual void Init();
		int ExecuteAction(Device::Action *); //TODO replace with enum
		//int ExecuteBehaviour(Tasks::Behaviour *);		// TODO replace with enum
		//int ExecuteRole(Tasks::Role*);
		//int ExecuteTask(Tasks::Task*);

		int Update(long delta);
		int RunAlgorithm(std::string info, void (*func_ptr)(...), ...);
		int Do();

		std::deque<Device::View*> GetSensorInfo();

		void SetSensorBlock(Device::SensorBlock*);
		void SetActuatorBlock(Device::ActuatorBlock*);

		Device::SensorBlock* GetSensorBlock();
		Device::ActuatorBlock* GetActuatorBlock();

		Device::DeviceTree* GetDevices();
		void SetDevices(Device::DeviceTree*);

		int PopulateSensorBlock();
		int PopulateActuatorBlock();

		UDPController* GetUDPController();
		void SetUDPController(UDPController*);

		SoccerServerCommandDict* GetCommandDict();
		void SetCommandDict(SoccerServerCommandDict*);

		Device::VoiceEmitter_UDPController* GetCommunicationDevice();
		void SetCommunicationDevice(Device::VoiceEmitter_UDPController*);


	};
}

