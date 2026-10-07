#pragma once
// A team of QuadSim UAVs with their MRS layers and an issuer on one simulated radio channel
// (spec 10 §4). The team tests and the batch runner fly on it.
#include <array>
#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "mrs/algorithms/MrsLayer.h"
#include "mrs/algorithms/TaskIssuer.h"
#include "mrs/device/Robot.h"
#include "mrs/protocol/Record.h"
#include "mrs/robot/RobotController.h"
#include "mrs/sim/QuadSim.h"

namespace MRS {
	namespace Sim {
		// One radio channel: every message reaches everyone else at the next tick.
		class Air {
		public:
			class Link : public Comm::ITransport {
			public:
				explicit Link(Air& air) : air_(air) {}
				bool Send(const std::string& text, const std::string&) override;
				std::vector<std::string> Poll() override;
				std::deque<std::string> inbox;

			private:
				Air& air_;
			};

			Link& Operator() { return op_; }
			void Add(QuadSim* sim) { sims_.push_back(sim); }
			void Deliver();
			long messages = 0, bytes = 0;

		private:
			Link op_{*this};
			std::vector<std::string> outbox_;
			std::vector<QuadSim*> sims_;
		};

		// The mission header (H_M, spec 06 §6).
		struct MissionSpec {
			std::string id = "m1";
			double lat0 = 56.9496, lon0 = 24.1052, alt0 = 10;
			double xmin = -100, xmax = 100, ymin = -100, ymax = 100, zmax = 60;
			double layer_base = 20, layer_step = 5, claim_grace = 120;
			std::vector<std::array<double, 3>> homes;  // robot k+1 starts at homes[k]
			std::string Text() const;
			Protocol::Record Header() const;
		};
		// n homes in a row along x, `spacing` apart and centred on x = 0, at y.
		std::vector<std::array<double, 3>> RowHomes(int n, double spacing = 4.0, double y = -5.0);

		// The robot files: the definition and port map texts of robot 1 and the behaviour library.
		struct RobotFiles {
			std::string definition;      // .mrsd text with `id 1`
			std::string port_map;        // .mrsp text
			std::string behaviours_path; // .mrsb file
		};
		// The definition of robot `id`, optionally without its LEDs (and their capability).
		std::string DefinitionFor(const std::string& base, int id, bool leds);
		std::string PortMapFor(const std::string& base, bool leds);

		using AllocatorFactory = std::function<std::unique_ptr<Algorithms::IAllocator>(int id)>;

		struct TeamConfig {
			MissionSpec mission;
			std::set<int> without_leds;
			AllocatorFactory allocator;                    // default: open MRS-RTA
			std::map<std::string, double> behaviour_priority;  // library entry name -> priority
			Algorithms::MrsConfig mrs;
			Algorithms::IssuerConfig issuer;
			QuadParams quad;                               // seed is offset by the robot id
			double separation = 1.0;                       // m: closer than this is a breach
		};

		// Times Select of the allocator it wraps (the decision time of spec 10 §5).
		class TimedAllocator : public Algorithms::IAllocator {
		public:
			explicit TimedAllocator(std::unique_ptr<Algorithms::IAllocator> inner) : inner_(std::move(inner)) {}
			void Bind(const Algorithms::AllocatorContext& ctx) override;
			void OnTaskReceived(const std::string& task, double t) override { inner_->OnTaskReceived(task, t); }
			void OnMessage(const Comm::Message& m, double t) override { inner_->OnMessage(m, t); }
			void OnTaskFinished(const std::string& task, Algorithms::PoolState s, double t) override { inner_->OnTaskFinished(task, s, t); }
			void OnCapabilityChanged(double t) override { inner_->OnCapabilityChanged(t); }
			Algorithms::Decision Select(const Environment::Worldview& w, const Algorithms::CurrentTask& current, double t) override;
			Algorithms::AllocatorInfo Info() const override { return inner_->Info(); }
			Algorithms::IAllocator& Inner() { return *inner_; }

			long calls = 0;
			double total_s = 0.0, worst_s = 0.0;

		private:
			std::unique_ptr<Algorithms::IAllocator> inner_;
		};

		struct TeamRobot {
			TeamRobot(int id, const std::array<double, 3>& home, bool leds, const RobotFiles& files, const TeamConfig& c);

			QuadSim sim;
			Device::DeviceRegistry devices;
			Task::FunctionRegistry functions;
			Device::Robot robot;
			std::unique_ptr<Robot::RobotController> ctl;
			std::unique_ptr<Comm::CommBlockTransport> transport;
			std::unique_ptr<Algorithms::MrsLayer> layer;
			TimedAllocator* timing = nullptr;  // owned by the layer
		};

		struct TeamMetrics {
			long separation_breaches = 0;  // pairs of airborne robots coming closer than `separation`
			double min_separation = 1e9;   // m, between airborne robots
			long fence_exits = 0;          // robots leaving the fence box
			long ticks = 0;
		};

		class Team {
		public:
			Team(const RobotFiles& files, const TeamConfig& c);

			double Time() const { return robots.front()->sim.Time(); }
			// Ticks everyone until the issuer is done or `seconds` pass. True when done.
			bool Run(double seconds, bool stop_when_done = true);
			int DoneBy(const std::string& name) const;

			Air air;
			Algorithms::TaskIssuer issuer;
			std::vector<std::unique_ptr<TeamRobot>> robots;
			TeamMetrics metrics;

		private:
			void Watch();
			TeamConfig c_;
			std::set<std::pair<int, int>> close_;
			std::set<int> outside_;
		};
	}
}
