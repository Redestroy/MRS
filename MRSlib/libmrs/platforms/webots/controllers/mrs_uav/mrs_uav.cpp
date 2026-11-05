// A UAV of the team (spec 09): the robot controller of spec 08 under the MRS layer with MRS-RTA.
//
// Usage (controllerArgs): <robot id> [rta|rta-x|sta|cbba|ldta2] [definition.mrsd] [ports.mrsp] [behaviours.mrsb]
//                         [--zones zones.csv] [--pad ID,X,Y,HOLD_X,HOLD_Y] [--drop-area X,Y,R] [--relay]
// Defaults: open MRS-RTA and mavic_webots.mrsd, mavic_webots.mrsp, uav_behaviours.mrsb next to the
// controller. The definition's head node id is replaced by <robot id>. The mission header and the
// tasks come from the issuer (mrs_issuer) over the radio; the robot waits on the ground until the
// header arrives. The Mavic needs an Emitter "emitter" and a Receiver "receiver" in a body slot, on
// the radio's channel, and WorldInfo needs gpsCoordinateSystem "WGS84".
//
// Delivery (spec 18): --zones reads the zones written by mrs_generate delivery --zones-out
// (id,x,y,z,radius,hold_x,hold_y); --pad adds a charging pad zone and turns recharging on (a Webots
// Charger at the pad charges the battery); --drop-area (repeatable) allows intermediate drops
// (§7); --relay sends on what the robot hears (§6.2). The sling's tag reader needs the Mavic's
// supervisor field TRUE.
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <regex>
#include <sstream>
#include <string>
#include <vector>
#include <cctype>

#include <webots/Supervisor.hpp>

#include "mrs/algorithms/Cbba.h"
#include "mrs/algorithms/Ldta2.h"
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

	std::vector<double> Numbers(const std::string& text) {
		std::vector<double> out;
		std::stringstream ls(text);
		std::string cell;
		while (std::getline(ls, cell, ',')) out.push_back(std::stod(cell));
		return out;
	}

	// Rows of numbers; a header line or a line starting with # is skipped.
	std::vector<std::vector<double>> Csv(const std::string& text) {
		std::vector<std::vector<double>> rows;
		std::stringstream in(text);
		std::string line;
		while (std::getline(in, line)) {
			if (!line.empty() && line.back() == '\r') line.pop_back();
			if (line.empty() || line[0] == '#' || !(std::isdigit(static_cast<unsigned char>(line[0])) || line[0] == '-')) continue;
			rows.push_back(Numbers(line));
		}
		return rows;
	}
}

int main(int argc, char** argv) {
	if (argc < 2) {
		std::fprintf(stderr, "usage: mrs_uav <robot id> [rta|rta-x|sta|cbba|ldta2] [definition.mrsd] [ports.mrsp] [behaviours.mrsb]\n");
		return 2;
	}
	std::setvbuf(stdout, nullptr, _IONBF, 0);  // Webots shows a controller's output only as it is flushed
	std::vector<std::string> pos;
	Algorithms::MrsConfig mrs;
	bool recharge = false;
	try {
		for (int k = 1; k < argc; ++k) {
			const std::string a = argv[k];
			if (a.rfind("--", 0) != 0) {
				pos.push_back(a);
				continue;
			}
			if (a == "--relay") {
				mrs.relay = true;
				continue;
			}
			if (k + 1 >= argc) throw std::runtime_error("missing value after " + a);
			const std::string v = argv[++k];
			if (a == "--zones") {
				for (const auto& row : Csv(ReadFile(v))) {
					if (row.size() < 7) continue;
					Algorithms::ZoneSpec z;
					z.id = static_cast<std::int64_t>(row[0]);
					z.x = row[1];
					z.y = row[2];
					z.z = row[3];
					z.radius = row[4];
					z.hold_x = row[5];
					z.hold_y = row[6];
					mrs.zones.push_back(z);
				}
			} else if (a == "--pad") {
				const auto n = Numbers(v);
				if (n.size() != 5) throw std::runtime_error("--pad expects ID,X,Y,HOLD_X,HOLD_Y");
				Algorithms::ZoneSpec z;
				z.id = static_cast<std::int64_t>(n[0]);
				z.x = n[1];
				z.y = n[2];
				z.radius = 1.5;
				z.hold_x = n[3];
				z.hold_y = n[4];
				z.pad = true;
				mrs.zones.push_back(z);
				recharge = true;
			} else if (a == "--drop-area") {
				const auto n = Numbers(v);
				if (n.size() != 3) throw std::runtime_error("--drop-area expects X,Y,R");
				mrs.drop_areas.push_back({n[0], n[1], n[2]});
			} else {
				throw std::runtime_error("unknown option " + a);
			}
		}
	} catch (const std::exception& e) {
		std::fprintf(stderr, "mrs_uav: %s\n", e.what());
		return 2;
	}
	if (pos.empty()) {
		std::fprintf(stderr, "usage: mrs_uav <robot id> [rta|rta-x|sta|cbba|ldta2] [definition.mrsd] [ports.mrsp] [behaviours.mrsb] [options]\n");
		return 2;
	}
	const int id = std::atoi(pos[0].c_str());
	const std::string mode = pos.size() > 1 ? pos[1] : "rta";
	const std::string definition_path = pos.size() > 2 ? pos[2] : "mavic_webots.mrsd";
	const std::string ports_path = pos.size() > 3 ? pos[3] : "mavic_webots.mrsp";
	const std::string behaviours_path = pos.size() > 4 ? pos[4] : "uav_behaviours.mrsb";

	// A Supervisor works as a plain Robot when the node's supervisor field is FALSE; the sling's tag
	// reader needs it TRUE.
	webots::Supervisor webots_robot;
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
		config.recharge = recharge;
		Robot::RobotController controller(robot, library, functions, config);
		std::ofstream journal("r" + std::to_string(id) + ".mrsj", std::ios::binary | std::ios::app);
		controller.SetJournal(&journal);

		Comm::CommBlockTransport transport(robot.blocks.comms);
		std::unique_ptr<Algorithms::IAllocator> allocator;
		if (mode == "sta") {
			allocator = std::make_unique<Algorithms::StaAllocator>();
		} else if (mode == "cbba") {
			allocator = std::make_unique<Algorithms::CbbaAllocator>();
		} else if (mode == "ldta2") {
			allocator = std::make_unique<Algorithms::Ldta2Allocator>();
		} else {
			Algorithms::RtaConfig rta;
			rta.exclusive = mode == "rta-x";
			allocator = std::make_unique<Algorithms::RtaAllocator>(rta);
		}
		Algorithms::MrsLayer layer(controller, transport, functions, std::move(allocator), mrs);

		std::string last, last_stack;
		double next_status = 0.0;
		while (platform.Step()) {
			const double t = platform.Time();
			layer.Tick(t);
			if (t >= next_status) {  // a status line every 5 s, to see what the robot hears
				next_status = t + 5.0;
				const auto& m = layer.Messages().Stats();
				std::printf("%8.2f r%d mission '%s' received %ld (bad %ld, other mission %ld) tasks %ld blocked %d stopped %d\n", t, id,
				            layer.Messages().Mission().c_str(), m.received, m.dropped_parse, m.dropped_mission, layer.Stats().tasks_received,
				            static_cast<int>(controller.TasksBlocked()), static_cast<int>(controller.Stopped()));
				const auto& w = controller.World();
				if (w.Bool("payload.held", t).value_or(false)) {
					auto g = [&](const char* f) { return w.Scalar(f, t).value_or(-999.0); };
					std::printf("%8.2f r%d package %.0f at (%.1f, %.1f, %.1f) goal (%.1f, %.1f, %.1f) load %.2f kg at_dest %d\n", t, id, g("payload.id"),
					            g("pose.enu.x"), g("pose.enu.y"), g("pose.enu.z"), g("payload.goal.x"), g("payload.goal.y"), g("payload.goal.z"),
					            g("sling.load"), static_cast<int>(w.Bool("payload.at_dest", t).value_or(false)));
					std::string stack;
					for (const auto& l : controller.Executor().StackLabels(true)) stack += l + " ";
					std::printf("%8.2f r%d running %s\n", t, id, stack.c_str());
				}
			}
			if (!layer.Assigned().empty()) {  // what ran last, to say where a task ended
				last_stack.clear();
				for (const auto& l : controller.Executor().StackLabels(true)) last_stack += l + " ";
			}
			if (layer.Assigned() != last) {
				if (!last.empty()) {
					const auto s = layer.StateOf(last);
					std::printf("%8.2f r%d task %s ended %s (done %ld, failed %ld, switches %ld, dumps %ld/%ld, cancels %ld) in: %s\n", t, id, last.c_str(),
					            s ? Algorithms::PoolStateName(*s) : "?", layer.Stats().done, layer.Stats().failed, layer.Stats().switches, layer.Stats().dumps_static, layer.Stats().dumps_dynamic, layer.Stats().cancels, last_stack.c_str());
				}
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
