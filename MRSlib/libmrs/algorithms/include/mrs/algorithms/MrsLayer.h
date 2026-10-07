#pragma once
// The MRS layer of one robot (plan §8, spec 09 §5): messages in and out, the task pool, the
// peer table, the dump rule, and the allocator that feeds the robot controller.
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "mrs/algorithms/Allocator.h"
#include "mrs/algorithms/TaskPool.h"
#include "mrs/comm/Messenger.h"
#include "mrs/robot/RobotController.h"
#include "mrs/task/TaskFactory.h"

namespace MRS {
	namespace Algorithms {
		struct MrsConfig {
			double state_period = 0.5;     // s between M_STATE (2 Hz, spec 06 §4)
			double profile_period = 10.0;  // s between M_PROFILE, so late joiners learn it
			double peer_timeout = 10.0;    // s (spec 06 §5)
			double retry_cooldown = 30.0;  // s after a dynamic dump (spec 06 §5)
			double claim_grace = 120.0;    // s, until a mission header says otherwise (spec 06 §4.1)
		};

		struct MrsStats {
			long tasks_received = 0, done = 0, failed = 0, dumps_static = 0, dumps_dynamic = 0;
			long switches = 0, cancels = 0;
		};

		class MrsLayer {
		public:
			MrsLayer(Robot::RobotController& robot, Comm::ITransport& transport, const Task::FunctionRegistry& registry,
			         std::unique_ptr<IAllocator> allocator, MrsConfig config = {},
			         std::unique_ptr<ISizeEstimator> size = nullptr);

			// One tick: messages, allocation, the robot controller's tick, then outgoing state.
			void Tick(double t);

			const std::string& Self() const { return self_; }
			TaskPool& Pool() { return pool_; }
			PeerTable& Peers() { return peers_; }
			Comm::Messenger& Messages() { return messenger_; }
			IAllocator& Allocator() { return *allocator_; }
			const MrsStats& Stats() const { return stats_; }
			// The task this layer gave the robot, or empty.
			const std::string& Assigned() const { return assigned_; }

		private:
			void Handle(const Comm::Message& m, double t);
			void OnTasks(const Comm::Message& m, double t);
			void OnFinished(const std::string& id, PoolState s, const std::string& by, double t);
			void Allocate(double t);
			void Assign(const std::string& id, double t);
			void Unassign(double t);
			void AfterRobot(double t);
			void SendState(double t);
			void SendProfile(double t);
			void SendDump(const std::string& id, const std::string& reason, bool is_static, long progress, double t);
			void CheckImpossible(PoolEntry& e, double t);
			bool StaticOk(const Task::Task& task) const;

			Robot::RobotController& robot_;
			Comm::Messenger messenger_;
			Task::TaskFactory factory_;
			std::unique_ptr<IAllocator> allocator_;
			std::unique_ptr<ISizeEstimator> size_;
			MrsConfig c_;
			std::string self_;
			TaskPool pool_;
			PeerTable peers_;
			std::string assigned_;
			std::deque<std::pair<Task::TaskEvent, double>> robot_events_;
			std::deque<Robot::RaisedEvent> raised_;
			double now_ = 0.0;
			double last_state_ = -1e9, last_profile_ = -1e9;
			MrsStats stats_;
		};
	}
}
