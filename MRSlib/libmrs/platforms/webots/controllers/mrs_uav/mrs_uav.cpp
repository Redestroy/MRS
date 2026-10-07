// A UAV of the team (spec 09): the robot controller of spec 08 under the MRS layer with MRS-RTA.
//
// Usage (controllerArgs): <robot id> [rta|rta-x] [definition.mrsd] [ports.mrsp] [behaviours.mrsb]
// Defaults: open MRS-RTA and mavic_webots.mrsd, mavic_webots.mrsp, uav_behaviours.mrsb next to the
// controller. The definition's head node id is replaced by <robot id>. The mission header and the
// tasks come from the issuer (mrs_issuer) over the radio; the robot waits on the ground until the
// header arrives. The Mavic needs an Emitter "emitter" and a Receiver "receiver" in a body slot, on
// the radio's channel, and WorldInfo needs gpsCoordinateSystem "WGS84".
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <regex>
#include <sstream>
#include <string>

#include <webots/Robot.hpp>

#include "mrs/algorithms/MrsLayer.h"
#include "mrs/device/Robot.h"
#include "mrs/device/uav/UavDevices.h"
#include "mrs/platform/WebotsPlatform.h"
#include "mrs/robot/RobotController.h"

using namespace MRS;

namespace {
	std::string ReadFile(const std::string& path) {
		std::ifstream in(path, std::ios::binary);
		if (!in) throw std::runtime_error("cannot open " + path);
		std::stringstream s;
		s << in.rdbuf();
		return s.str();
	}
}

int main(int argc, char** argv) {
	if (argc < 2) {
		std::fprintf(stderr, "usage: mrs_uav <robot id> [rta|rta-x] [definition.mrsd] [ports.mrsp] [behaviours.mrsb]\n");
		return 2;
	}
	const int id = std::atoi(argv[1]);
	const std::string mode = argc > 2 ? argv[2] : "rta";
	const std::string definition_path = argc > 3 ? argv[3] : "mavic_webots.mrsd";
	const std::string ports_path = argc > 4 ? argv[4] : "mavic_webots.mrsp";
	const std::string behaviours_path = argc > 5 ? argv[5] : "uav_behaviours.mrsb";

	webots::Robot webots_robot;
	Platform::WebotsPlatform platform(webots_robot);
	try {
		Device::DeviceRegistry devices;
		Device::Uav::RegisterUavDevices(devices);
		Task::FunctionRegistry functions;
		Task::RegisterUavFunctions(functions);
		Task::TaskFactory factory(functions);
		Task::BehaviourLibrary library;
		library.PopulateFromFile(behaviours_path, factory);

		std::string definition = std::regex_replace(ReadFile(definition_path), std::regex(" id [0-9]+ "), " id " + std::to_string(id) + " ");
		const auto ports = Port::PortMap::Parse(ReadFile(ports_path));
		Device::Robot robot = Device::BuildRobot(definition, devices, platform, &ports);
		for (const auto& f : robot.faults) std::printf("r%d fault: %s (%s)\n", id, f.node.c_str(), f.reason.c_str());

		Robot::ControllerConfig config;
		config.robot = "r" + std::to_string(id);
		config.robot_id = id;
		Robot::RobotController controller(robot, library, functions, config);
		std::ofstream journal("r" + std::to_string(id) + ".mrsj", std::ios::binary | std::ios::app);
		controller.SetJournal(&journal);

		Comm::CommBlockTransport transport(robot.blocks.comms);
		Algorithms::RtaConfig rta;
		rta.exclusive = mode == "rta-x";
		Algorithms::MrsLayer layer(controller, transport, functions, std::make_unique<Algorithms::RtaAllocator>(rta));

		std::string last;
		while (platform.Step()) {
			const double t = platform.Time();
			layer.Tick(t);
			if (layer.Assigned() != last) {
				last = layer.Assigned();
				std::printf("%8.2f r%d task %s\n", t, id, last.empty() ? "-" : last.c_str());
			}
		}
	} catch (const std::exception& e) {
		std::fprintf(stderr, "r%d: %s\n", id, e.what());
		return 1;
	}
	return 0;
}
