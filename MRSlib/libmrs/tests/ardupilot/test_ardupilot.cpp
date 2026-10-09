// WP10 tests (spec 14 §7, plan §11 WP10): the ArduPilot platform, the GUIDED-mode flight control
// unit and UDP, against the in-process mock autopilot (CI has no SITL). The same task strings and
// the same allocator as on Webots and QuadSim.
#include <cmath>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "doctest.h"
#include "../protocol/TestFiles.h"
#include "mrs/algorithms/MrsLayer.h"
#include "mrs/algorithms/TaskIssuer.h"
#include "mrs/device/Robot.h"
#include "mrs/device/uav/UavDevices.h"
#include "mrs/platform/ArduPilotPlatform.h"
#include "mrs/platform/GuidedFcu.h"
#include "mrs/platform/MockAutopilot.h"
#include "mrs/platform/UdpTransport.h"
#include "mrs/protocol/Parser.h"
#include "mrs/robot/RobotController.h"
#include "mrs/world/GeoReference.h"

using namespace MRS;

namespace {
	const double kLat0 = 56.9496, kLon0 = 24.1052, kAlt0 = 10.0;

	// Fence ±100 m, 60 m high, layers from 20 m every 5 m; robot k's home at (10 k, 5, 0), so the
	// autopilot's local origin is not the mission's.
	std::string Header(int n) {
		std::ostringstream s;
		s << "H: H_M m1 " << kLat0 << " " << kLon0 << " " << kAlt0 << " -100 100 -100 100 60 20 5 120 " << n;
		for (int k = 1; k <= n; ++k) s << " " << k << " " << 10 * k << " 5 0";
		s << "/";
		return s.str();
	}

	Protocol::Record Record(const std::string& text) {
		auto parsed = Protocol::Parse(text);
		REQUIRE(parsed.Ok());
		return parsed.document.records.at(0);
	}

	Platform::MockAutopilotConfig AutopilotAt(int k) {
		Platform::MockAutopilotConfig c;
		c.system_id = k;
		Environment::GeoReference geo(kLat0, kLon0, kAlt0);
		geo.ToGeodetic({10.0 * k, 5.0, 0.0}, c.home_lat, c.home_lon, c.home_alt);
		return c;
	}

	// One robot on a mock autopilot, as mrs_ardupilot_uav builds it.
	struct Copter {
		explicit Copter(int k) : Copter(k, AutopilotAt(k)) {}
		Copter(int k, Platform::MockAutopilotConfig config) : ap(config) {
			Platform::ArduPilotConfig pc;
			pc.clock = Platform::ClockMode::MANUAL;
			pc.system_id = 245 - k;
			platform = std::make_unique<Platform::ArduPilotPlatform>(pc, ap.TakeLink());
			Device::Uav::RegisterUavDevices(devices);
			Platform::RegisterArduPilotDevices(devices);
			Task::RegisterUavFunctions(functions);
			Task::TaskFactory factory(functions);
			library.PopulateFromFile((Test::ExamplesDir() / "uav_behaviours.mrsb").string(), factory);
			const auto map = Port::PortMap::Parse(Test::ReadFile(Test::ExamplesDir() / "quad_ardupilot.mrsp"));
			std::string def = Test::ReadFile(Test::ExamplesDir() / "quad_ardupilot.mrsd");
			const auto at = def.find(" id 1 ");
			REQUIRE(at != std::string::npos);
			def.replace(at, 6, " id " + std::to_string(k) + " ");
			robot = Device::BuildRobot(def, devices, *platform, &map);
			Robot::ControllerConfig cc;
			cc.robot = "r" + std::to_string(k);
			cc.robot_id = k;
			ctl = std::make_unique<Robot::RobotController>(robot, library, functions, cc);
		}

		void Step() {
			ap.Step(platform->Config().period);
			platform->Step();
		}
		double Time() const { return platform->Time(); }
		// Mission ENU of the mock's local NED (home at (10 k, 5, 0)).
		double X(int k) const { return ap.Position()[1] + 10.0 * k; }
		double Y() const { return ap.Position()[0] + 5.0; }
		double Z() const { return -ap.Position()[2]; }
		Platform::GuidedFcu* Fcu() const { return dynamic_cast<Platform::GuidedFcu*>(ctl->Fcu()); }

		Platform::MockAutopilot ap;
		std::unique_ptr<Platform::ArduPilotPlatform> platform;
		Device::DeviceRegistry devices;
		Task::FunctionRegistry functions;
		Task::BehaviourLibrary library;
		Device::Robot robot;
		std::unique_ptr<Robot::RobotController> ctl;
	};

	// A shared channel: every message reaches every other end.
	struct Hub {
		class End : public Comm::ITransport {
		public:
			End(Hub& hub) : hub_(hub) {}
			bool Send(const std::string& text, const std::string&) override {
				for (auto* e : hub_.ends)
					if (e != this) e->inbox.push_back(text);
				return true;
			}
			std::vector<std::string> Poll() override {
				auto out = inbox;
				inbox.clear();
				return out;
			}
			std::vector<std::string> inbox;

		private:
			Hub& hub_;
		};
		End& Add() {
			owned.push_back(std::make_unique<End>(*this));
			ends.push_back(owned.back().get());
			return *owned.back();
		}
		std::vector<std::unique_ptr<End>> owned;
		std::vector<End*> ends;
	};
}

TEST_CASE("the ArduPilot platform reads the autopilot's estimate in the library's frames") {
	Copter c(1);
	for (int k = 0; k < 50; ++k) c.Step();
	const auto& s = c.platform->State();
	REQUIRE(s.heartbeat);
	CHECK(c.platform->Config().target_system == 1);
	CHECK(s.global_seq > 0);
	CHECK(s.att_seq > 0);
	CHECK(s.remaining == doctest::Approx(1.0));
	// GLOBAL_POSITION_INT and the GUIDED status, through their ports.
	auto open = [&](const char* address) {
		Port::PortAssignment a;
		a.type = Port::PortType::MAVLINK;
		a.address = address;
		auto p = c.platform->Open(a);
		REQUIRE(p);
		return p;
	};
	auto gnss = open("GLOBAL_POSITION_INT");
	std::vector<double> v;
	REQUIRE(gnss->Read(v));
	Environment::GeoReference geo(kLat0, kLon0, kAlt0);
	const auto enu = geo.ToEnu(v[0], v[1], v[2]);
	CHECK(enu.x == doctest::Approx(10.0).epsilon(0.01));
	CHECK(enu.y == doctest::Approx(5.0).epsilon(0.01));
	CHECK(std::fabs(enu.z) < 0.01);
	CHECK_FALSE(gnss->Read(v));  // once per message
	auto att = open("ATTITUDE");
	REQUIRE(att->Read(v));
	CHECK(v[2] == doctest::Approx(3.14159265 / 2));  // NED yaw 0 (North) is ENU yaw π/2
	auto guided = open("GUIDED");
	REQUIRE(guided->Read(v));
	CHECK(v.at(0) == 1.0);  // alive
	CHECK(v.at(1) == 0.0);  // disarmed
	CHECK(open("SERVO:1")->Write({100.0}) == false);  // the autopilot drives the motors
	CHECK(c.platform->Scan().size() >= 7);
}

TEST_CASE("one copter on ArduPilot flies liftoff, fly-to, LED flash and landing from the same task strings") {
	Copter c(1);
	REQUIRE(c.robot.faults.empty());
	REQUIRE(c.Fcu());
	c.ctl->SetMission(Record(Header(1)));
	REQUIRE(c.ctl->AddTasks(Test::ReadFile(Test::ExamplesDir() / "uav_single_flight.mrst")) == 3);
	double top = 0.0;
	bool done = false;
	while (c.Time() < 200.0 && !done) {
		c.ctl->Tick(c.Time());
		top = std::max(top, c.Z());
		done = c.ctl->ListDone();
		c.Step();
	}
	REQUIRE(done);
	// The autopilot disarms a moment after touchdown; the fcu follows.
	for (const double end = c.Time() + 3.0; c.Time() < end;) {
		c.ctl->Tick(c.Time());
		c.Step();
	}
	for (const char* id : {"sf.1", "sf.2", "sf.3"}) {
		CAPTURE(id);
		CHECK(c.ctl->StateOf(id) == Task::TaskState::SUCCEEDED);
	}
	// sf.2's target is (40, 20, 10): flown in the mission frame although the autopilot's origin is (10, 5, 0).
	CHECK(std::hypot(c.X(1) - 40.0, c.Y() - 20.0) < 1.5);
	CHECK(top > 9.0);
	CHECK(c.Z() < 0.05);
	CHECK_FALSE(c.ap.Armed());
	CHECK_FALSE(c.Fcu()->Armed());
	CHECK(c.ap.Targets() > 10);  // GUIDED position targets
	CHECK(c.ap.MaxSpeedSeen() <= 8.0 + 0.1);  // max_speed_xy, sent as DO_CHANGE_SPEED
}

TEST_CASE("a takeoff that cannot arm fails after arm_timeout") {
	auto config = AutopilotAt(1);
	config.ready_after = 1e9;  // pre-arm checks that never pass
	Copter c(1, config);
	c.ctl->SetMission(Record(Header(1)));
	REQUIRE(c.ctl->AddTasks("T: T_A to.1 1 0 C_1 C_2 A_1..2/\nC_1: C_N/\nC_2: C_N/\nA_1: A_TO 5 1.5/\nA_2: A_N/\n") == 1);
	while (c.Time() < 90.0 && c.ctl->StateOf("to.1") != Task::TaskState::FAILED) {
		c.ctl->Tick(c.Time());
		c.Step();
	}
	CHECK(c.ctl->StateOf("to.1") == Task::TaskState::FAILED);
	CHECK(c.Time() > 60.0);  // arm_timeout
	CHECK_FALSE(c.ap.Armed());
	CHECK(c.ap.Mode() == Platform::CopterMode::GUIDED);  // it set GUIDED, then kept asking to arm
}

TEST_CASE("two copters on ArduPilot share a task set through the MRS layer with MRS-RTA-X") {
	Hub hub;
	std::vector<std::unique_ptr<Copter>> copters;
	std::vector<std::unique_ptr<Algorithms::MrsLayer>> layers;
	for (int k = 1; k <= 2; ++k) {
		copters.push_back(std::make_unique<Copter>(k));
		REQUIRE(copters.back()->robot.faults.empty());
		Algorithms::RtaConfig rta;
		rta.exclusive = true;
		layers.push_back(std::make_unique<Algorithms::MrsLayer>(*copters.back()->ctl, hub.Add(), copters.back()->functions,
		                                                         std::make_unique<Algorithms::RtaAllocator>(rta)));
	}
	Algorithms::TaskIssuer issuer(hub.Add(), Record(Header(2)));
	// Four points, two near each home; LEDs on the way.
	std::ostringstream tl;
	tl << "@: MRS 0.1/\n";
	const double pts[4][2] = {{0, 40}, {10, 50}, {40, 40}, {30, 50}};
	for (int k = 0; k < 4; ++k)
		tl << "L: L_D " << 1 + k << " T_1/\nT_1: T_A op." << k + 1 << " 10 0 C_1 C_2 A_1..3/\nC_1: C_P3 " << pts[k][0] << " " << pts[k][1]
		   << " 15 1 0.5 0 -1/\nC_2: C_N/\nA_1: A_L 3 65280/\nA_2: A_W 1/\nA_3: A_L 3 0/\n";
	issuer.LoadTimeline(tl.str());
	double t = 0.0;
	while (t < 400.0 && !issuer.Done()) {
		t = copters[0]->Time();
		issuer.Tick(t);
		for (auto& l : layers) l->Tick(t);
		for (auto& c : copters) c->Step();
	}
	REQUIRE(issuer.Done());
	std::set<std::string> by;
	for (const auto& entry : issuer.Tasks()) {
		const auto& it = entry.second;
		CAPTURE(entry.first);
		CHECK(it.done);
		CHECK(it.done_count == 1);
		by.insert(it.done_by);
	}
	CHECK(by.size() == 2);  // both copters worked
	REQUIRE(issuer.Makespan());
	MESSAGE("two ArduPilot copters, makespan " << *issuer.Makespan() << " s");
}

TEST_CASE("UDP transport: every message reaches every peer, as a broadcast") {
	Net::UdpTransportConfig a, b;
	a.port = 47611;
	b.port = 47612;
	a.peers = {{"127.0.0.1", 47612}};
	b.peers = {{"127.0.0.1", 47611}};
	Net::UdpTransport ta(a), tb(b);
	REQUIRE(ta.Ok());
	REQUIRE(tb.Ok());
	REQUIRE(ta.Send("M: M_DONE 0.1 m1 r1 all 1 1 op.1/", "all"));
	std::vector<std::string> got;
	for (int k = 0; k < 100 && got.empty(); ++k) {
		tb.Wait(0.01);
		got = tb.Poll();
	}
	REQUIRE(got.size() == 1);
	CHECK(got[0] == "M: M_DONE 0.1 m1 r1 all 1 1 op.1/");
	CHECK_FALSE(ta.Send(std::string(70000, 'x'), "all"));  // larger than one datagram
}

TEST_CASE("link URLs parse") {
	CHECK(Net::ParseEndpoint("127.0.0.1:5760").port == 5760);
	CHECK(Net::ParseEndpoint("14550").host == "127.0.0.1");
	CHECK_THROWS(Net::ParseEndpoint("host:port"));
	CHECK_THROWS(Net::OpenLink("serial:/dev/ttyUSB0"));
	auto udp = Net::OpenLink("udpin:0");
	CHECK_FALSE(udp->Connected());  // no sender yet
}
