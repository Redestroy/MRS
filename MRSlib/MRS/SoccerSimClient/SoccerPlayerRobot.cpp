#include "SoccerPlayerRobot.h"
namespace MRS {
	Device::DeviceTree* SoccerPlayerRobot::GetDevices()
	{
		return devices;
	}
	void SoccerPlayerRobot::SetDevices(Device::DeviceTree* dev)
	{
		this->devices = dev;
	}

	int SoccerPlayerRobot::PopulateSensorBlock()
	{
		/*
		for (auto i : this->GetDevices()) {
			if (i.GetDeviceType() == MRS::Device::DeviceType::SENSOR) {
				sensors.Add((Device::Sensor*)i);
			}
		}
		return sensors.size();
		*/
		return -1;
	}

	int SoccerPlayerRobot::PopulateActuatorBlock()
	{
		/*
		for (auto i : this->GetDevices()) {
			if (i.GetDeviceType() == MRS::Device::DeviceType::ACTUATOR) {
				actuators.Add((Device::Actuator*)i);
			}
		}
		return actuators.size();
		*/
		return -1;
	}

	UDPController* SoccerPlayerRobot::GetUDPController()
	{
		return udpController;
	}

	void SoccerPlayerRobot::SetUDPController(UDPController* ptr)
	{
		udpController = ptr;
	}

	SoccerServerCommandDict* SoccerPlayerRobot::GetCommandDict()
	{
		return cmd_dict;
	}

	void SoccerPlayerRobot::SetCommandDict(SoccerServerCommandDict* dict)
	{
		this->cmd_dict = dict;
	}

	Device::VoiceEmitter_UDPController* SoccerPlayerRobot::GetCommunicationDevice()
	{
		return com_device;
	}

	void SoccerPlayerRobot::SetCommunicationDevice(Device::VoiceEmitter_UDPController* com_device)
	{
		this->com_device = com_device;
	}

	int SoccerPlayerRobot::Update(long delta)
	{
		std::cout << "Updating robot\n";
		udpController->OnUpdate(delta);
		std::cout << "Updating sensors\n";
		sensors->Update(delta);
		return 0;
	}

	int SoccerPlayerRobot::Do()
	{
		//TODO: put safeguards and limits for actuators
		int sum_resp = 0;
		for (auto a : *actions) {
			if(actuators->DoAction(a)) sum_resp++;
		}
		return sum_resp;
	}

	void SoccerPlayerRobot::SetSensorBlock(Device::SensorBlock* sensors)
	{
		this->sensors = sensors;
	}

	void SoccerPlayerRobot::SetActuatorBlock(Device::ActuatorBlock* actuators)
	{
		this->actuators = actuators;
	}

	SoccerPlayerRobot::SoccerPlayerRobot()
	{
	}

	SoccerPlayerRobot::SoccerPlayerRobot(std::string ip_addr, unsigned short port, std::string team) : SoccerPlayerRobot(ip_addr, port, team, false)
	{
	}

	SoccerPlayerRobot::SoccerPlayerRobot(std::string ip_addr, unsigned short port, std::string team, bool goalie) : args(), server_parameters(), player_parameters(), player_type_parameters()
	{
		if (goalie) role = TEAM_ROLE::GOALIE;
		else role = TEAM_ROLE::NOT_ASSIGNED;
		team_name = "";
		team_name += team;
		udpController = new UDPController(ip_addr, port);
	}

	SoccerPlayerRobot::SoccerPlayerRobot(std::string ip_addr, unsigned short port, std::string team, std::string goalie) : SoccerPlayerRobot(ip_addr, port, team, goalie.compare("goalie")==0)
	{
	}
	SoccerPlayerRobot::~SoccerPlayerRobot()
	{
		delete udpController;
	}

	void SoccerPlayerRobot::Init()
	{
		//TODO make iterator
		/*
		for (s : sensors) {
			if (s.GetTag("ComType").compare("UDPSensor") == 0) {
				udpController->AttatchSignal((UDPPacketReceiver *)&s);
			}
		}*/
		sensors->Init(); //or can loop through tree and init each device seperetly
		actuators->Init();
		com_device->Init();
		//send init command
		// TODO create a way to create command arg structures
		args.clear();
		args.emplace_back("MRS_ver1"); //team name
		args.emplace_back("15"); //version
		if(role == TEAM_ROLE::GOALIE) args.emplace_back("(goalie)"); //goalie
		std::cout << "Command template in init " <<cmd_dict->GetCommandTemplate("init") << "\n";
		std::cout << "Args in init " << cmd_dict->ParseArgsIntoTemplate((cmd_dict->GetArgsTemplate("init")), args) << "\n";
		udpController->SendCommand(cmd_dict->GetCommandTemplate("init"),cmd_dict->ParseArgsIntoTemplate((cmd_dict->GetArgsTemplate("init")), args)); //Send Init command
		std::string init_response = udpController->GetPacket(); //Send Init command
		// create timer
		std::string packet = "";
		int init_count = 0;
		bool got_server_params = false, got_player_params = false, got_player_types = false;
		while (!(got_server_params && got_player_params && got_player_types)) {
			//delay
			//GetPacket
			packet = udpController->GetPacket(std::chrono::milliseconds(20));
			std::cout << "Init packet: " << packet << "\n";
			if (cmd_dict->ParsePacketId(packet).compare("init")==0) {
				// init received again
				std::cout << "Received init again: count " << init_count << "\n";
			}
			else if(cmd_dict->ParsePacketId(packet).compare("server_param") == 0 && !got_server_params) {
				server_parameters = cmd_dict->ParseResponse_ServerParameters(packet);
				got_server_params = true;
			}
			else if(cmd_dict->ParsePacketId(packet).compare("player_param") == 0 && !got_player_params) {
				player_parameters = cmd_dict->ParseResponse_PlayerParameters(packet);
				got_player_params = true;
			}
			else if(cmd_dict->ParsePacketId(packet).compare("player_type") == 0 && !got_player_types) {
				//
				player_type_parameters.emplace(cmd_dict->ParsePlayerId(packet), std::map<std::string, std::string>(cmd_dict->ParseResponse_PlayerTypeParameters(packet)));
				got_player_types = true;
			}
			else {
				// received sensor info so init is finished
				break;
			}
		}
		std::cout << "Init response: " << init_response << "\n";
		std::cout << "Player types: " << init_response << "\n";
		std::cout << "Player params: " << init_response << "\n";
		//ParseResponse()
		std::map<std::string, std::string> response_map = cmd_dict->ParseResponse_Init(init_response);
		
		for (auto p : response_map) {
			std::cout << "Response map content: " << p.first << " " << p.second << "\n";
		}

		side = response_map["side"].compare("r") == 0;

		try {
			id = std::stoi(response_map["id"]);
		}
		catch (...) {
			std::cout << "Couldn't parse id\n";
		}

		if (response_map["game_phase"].compare("before_kick_off") == 0) {
			phase = GAME_PHASE::BEFORE_KICK_OFF;
		}
	}
}