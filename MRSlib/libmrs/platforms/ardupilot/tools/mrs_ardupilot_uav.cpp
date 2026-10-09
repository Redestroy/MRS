// A UAV of the team on ArduPilot (spec 14 §8): the robot controller and the MRS layer of mrs_uav,
// on ArduPilotPlatform, talking to the other agents over UDP.
//
// Usage: mrs_ardupilot_uav <robot id> [options]
//   --alloc rta|rta-x|sta|cbba|ldta2   allocator (default rta)
//   --link URL          autopilot link (default tcp:127.0.0.1:<5760 + 10 × (id − 1)>, SITL instance id − 1)
//   --def FILE --ports FILE --behaviours FILE   (default quad_ardupilot.mrsd, quad_ardupilot.mrsp,
//                       uav_behaviours.mrsb in the spec examples)
//   --port P            UDP port this robot listens on (default 14600 + id)
//   --peers H:P,H:P     where messages go (default 127.0.0.1:14600 .. 14600 + --team)
//   --team N            team size for the default peers (default 8)
//   --multicast GROUP   use a multicast group on --port instead of peers
//   --epoch UNIX_S      mission time 0 (default: the last UTC midnight); every agent needs the same
//   --clock wall|autopilot   (default wall)
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <regex>
#include <sstream>
#include <string>

#include "mrs/algorithms/Cbba.h"
#include "mrs/algorithms/Ldta2.h"
#include "mrs/algorithms/MrsLayer.h"
#include "mrs/device/Robot.h"
#include "mrs/device/uav/UavDevices.h"
#include "mrs/platform/ArduPilotPlatform.h"
#include "mrs/platform/GuidedFcu.h"
#include "mrs/platform/UdpTransport.h"
#include "mrs/robot/RobotController.h"

#ifndef MRS_SPEC_EXAMPLES
#define MRS_SPEC_EXAMPLES "."
#endif

using namespace MRS;

namespace {
	Platform::ArduPilotPlatform* g_platform = nullptr;
	void OnSignal(int) {
		if (g_platform) g_platform->Stop();
	}

	std::string ReadFile(const std::string& path) {
		std::ifstream in(path, std::ios::binary);
		if (!in) throw std::runtime_error("cannot open " + path);
		std::stringstream s;
		s << in.rdbuf();
		return s.str();
	}

	std::vector<std::string> Split(const std::string& text, char sep) {
		std::vector<std::string> out;
		std::stringstream s(text);
		std::string item;
		while (std::getline(s, item, sep))
			if (!item.empty()) out.push_back(item);
		return out;
	}
}

int main(int argc, char** argv) {
	if (argc < 2) {
		std::fprintf(stderr, "usage: mrs_ardupilot_uav <robot id> [--alloc A] [--link URL] [--def F] [--ports F] [--behaviours F]\n"
		                     "       [--port P] [--peers H:P,..] [--team N] [--multicast G] [--epoch S] [--clock wall|autopilot]\n");
		return 2;
	}
	std::setvbuf(stdout, nullptr, _IONBF, 0);
	const int id = std::atoi(argv[1]);
	std::string alloc = "rta", link, def = std::string(MRS_SPEC_EXAMPLES) + "/quad_ardupilot.mrsd",
	            ports_path = std::string(MRS_SPEC_EXAMPLES) + "/quad_ardupilot.mrsp",
	            behaviours = std::string(MRS_SPEC_EXAMPLES) + "/uav_behaviours.mrsb", peers, multicast, clock = "wall";
	int port = 14600 + id, team = 8;
	double epoch = -1.0;
	for (int k = 2; k + 1 < argc; k += 2) {
		const std::string key = argv[k], value = argv[k + 1];
		if (key == "--alloc") alloc = value;
		else if (key == "--link") link = value;
		else if (key == "--def") def = value;
		else if (key == "--ports") ports_path = value;
		else if (key == "--behaviours") behaviours = value;
		else if (key == "--port") port = std::atoi(value.c_str());
		else if (key == "--peers") peers = value;
		else if (key == "--team") team = std::atoi(value.c_str());
		else if (key == "--multicast") multicast = value;
		else if (key == "--epoch") epoch = std::atof(value.c_str());
		else if (key == "--clock") clock = value;
		else {
			std::fprintf(stderr, "unknown option %s\n", key.c_str());
			return 2;
		}
	}
	if (link.empty()) link = "tcp:127.0.0.1:" + std::to_string(5760 + 10 * (id - 1));

	try {
		Platform::ArduPilotConfig pc;
		pc.link = link;
		pc.epoch = epoch;
		pc.system_id = 245 - id;  // one companion id per robot, away from the autopilots' 1..
		pc.clock = clock == "autopilot" ? Platform::ClockMode::AUTOPILOT : Platform::ClockMode::WALL;
		Platform::ArduPilotPlatform platform(pc);
		g_platform = &platform;
		std::signal(SIGINT, OnSignal);
		std::signal(SIGTERM, OnSignal);
		std::printf("r%d: waiting for the autopilot on %s\n", id, link.c_str());
		if (!platform.WaitForAutopilot(120.0)) throw std::runtime_error("no autopilot heartbeat and position on " + link);
		std::printf("r%d: autopilot %d found at %.7f %.7f %.1f m\n", id, platform.Config().target_system, platform.State().lat,
		            platform.State().lon, platform.State().alt_amsl);

		Device::DeviceRegistry devices;
		Device::Uav::RegisterUavDevices(devices);
		Platform::RegisterArduPilotDevices(devices);
		Task::FunctionRegistry functions;
		Task::RegisterUavFunctions(functions);
		Task::TaskFactory factory(functions);
		Task::BehaviourLibrary library;
		library.PopulateFromFile(behaviours, factory);

		const std::string definition = std::regex_replace(ReadFile(def), std::regex(" id [0-9]+ "), " id " + std::to_string(id) + " ");
		const auto port_map = Port::PortMap::Parse(ReadFile(ports_path));
		Device::Robot robot = Device::BuildRobot(definition, devices, platform, &port_map);
		for (const auto& f : robot.faults) std::printf("r%d fault: %s (%s)\n", id, f.node.c_str(), f.reason.c_str());

		Robot::ControllerConfig config;
		config.robot = "r" + std::to_string(id);
		config.robot_id = id;
		Robot::RobotController controller(robot, library, functions, config);
		std::ofstream journal("r" + std::to_string(id) + ".mrsj", std::ios::binary | std::ios::app);
		controller.SetJournal(&journal);

		Net::UdpTransportConfig uc;
		uc.port = port;
		uc.multicast = multicast;
		if (multicast.empty()) {
			if (peers.empty())
				for (int k = 0; k <= team; ++k) uc.peers.push_back({"127.0.0.1", 14600 + k});
			else
				for (const auto& p : Split(peers, ',')) uc.peers.push_back(Net::ParseEndpoint(p));
		}
		Net::UdpTransport transport(uc);
		if (!transport.Ok()) throw std::runtime_error("cannot open UDP port " + std::to_string(port));

		std::unique_ptr<Algorithms::IAllocator> allocator;
		if (alloc == "sta") allocator = std::make_unique<Algorithms::StaAllocator>();
		else if (alloc == "cbba") allocator = std::make_unique<Algorithms::CbbaAllocator>();
		else if (alloc == "ldta2") allocator = std::make_unique<Algorithms::Ldta2Allocator>();
		else {
			Algorithms::RtaConfig rta;
			rta.exclusive = alloc == "rta-x";
			allocator = std::make_unique<Algorithms::RtaAllocator>(rta);
		}
		Algorithms::MrsLayer layer(controller, transport, functions, std::move(allocator));

		std::string last;
		double next_status = 0.0;
		while (platform.Step()) {
			const double t = platform.Time();
			layer.Tick(t);
			if (t >= next_status) {
				next_status = t + 5.0;
				const auto& s = platform.State();
				std::printf("%10.2f r%d mission '%s' tasks %ld armed %d mode %u rel_alt %.1f battery %.0f%%\n", t, id,
				            layer.Messages().Mission().c_str(), layer.Stats().tasks_received, s.armed ? 1 : 0, s.custom_mode, s.rel_alt,
				            s.remaining * 100);
			}
			if (layer.Assigned() != last) {
				last = layer.Assigned();
				std::printf("%10.2f r%d task %s\n", t, id, last.empty() ? "-" : last.c_str());
			}
		}
	} catch (const std::exception& e) {
		std::fprintf(stderr, "r%d: %s\n", id, e.what());
		return 1;
	}
	return 0;
}
