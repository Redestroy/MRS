#pragma once
// The MRS layer of one robot (plan §8, spec 09 §5): messages in and out, the task pool, the
// peer table, the dump rule, and the allocator that feeds the robot controller.
#include <deque>
#include <map>
#include <optional>
#include <set>
#include <memory>
#include <string>
#include <vector>

#include "mrs/algorithms/Allocator.h"
#include "mrs/algorithms/TaskPool.h"
#include "mrs/algorithms/TaskTree.h"
#include "mrs/comm/Messenger.h"
#include "mrs/robot/RobotController.h"
#include "mrs/task/TaskFactory.h"

namespace MRS {
	namespace Algorithms {
		// One entry of the sync list (spec 15 §2.1): world fields under `prefix`, every `period` s.
		struct SyncEntry {
			std::string prefix;
			double period = 1.0;
		};

		struct MrsConfig {
			double state_period = 0.5;     // s between M_STATE (2 Hz, spec 06 §4)
			double profile_period = 10.0;  // s between M_PROFILE, so late joiners learn it
			double peer_timeout = 10.0;    // s (spec 06 §5)
			double retry_cooldown = 30.0;  // s after a dynamic dump (spec 06 §5)
			double claim_grace = 120.0;    // s, until a mission header says otherwise (spec 06 §4.1)
			// Tree tasks are split into leaves that any robot may take (spec 11 §2). Off: the robot
			// runs each tree itself as one complex task (spec 03 §6.1), as a lone robot may.
			bool split_trees = true;
			// The shared channel's bitrate, bit/s; 0: no limit known (spec 13 §3). Each agent's share
			// is bitrate / (live peers + this robot + the issuer); heartbeats take at most
			// state_share of it, so M_STATE slows down on a slow link.
			double bitrate_bps = 0.0;
			double state_share = 0.25;
			// Worldview sync (spec 15 §2): what this robot publishes, how far (m; 0: no limit) and
			// at most which share of its bitrate.
			std::vector<SyncEntry> sync{{"det.", 1.0}};
			double sync_range = 0.0;
			double sync_share = 0.25;
		};

		struct MrsStats {
			long tasks_received = 0, done = 0, failed = 0, dumps_static = 0, dumps_dynamic = 0;
			long switches = 0, cancels = 0;
			long trees = 0;  // tree tasks decomposed (spec 11 §2)
			long sync_sent = 0, sync_views_sent = 0, sync_views_received = 0, info_requests = 0;  // spec 15 §2
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
			// The pool state of a task, a leaf or a node of a tree task (spec 03 §3.2).
			std::optional<PoolState> StateOf(const std::string& id) const;
			// The tree a leaf or node belongs to, or nullptr.
			const TaskTree* TreeOf(const std::string& id) const;

		private:
			void Handle(const Comm::Message& m, double t);
			void OnTasks(const Comm::Message& m, double t);
			void AddTree(const Protocol::Record& root, double t);
			void AddEntry(std::shared_ptr<const Task::Task> task, double t);
			void UpdateTrees(double t);
			struct Tree;
			bool Gated(const Tree& tr, const TreeNode& unit, double t) const;
			PoolState UnitState(const Tree& tr, const std::string& id) const;
			void OnFinished(const std::string& id, PoolState s, const std::string& by, double t);
			void Allocate(double t);
			void Assign(const std::string& id, double t);
			void Unassign(double t);
			void AfterRobot(double t);
			void SendState(double t);
			double Budget(double t) const;
			void Redone(const std::string& id, double t);  // M_DONE again for a repeated task we did  // this robot's share of the channel, bit/s; 0: no limit
			void SendProfile(double t);
			// Worldview sync (spec 15 §2, MrsSync.cpp).
			void SendSync(double t);
			void OnShared(const Comm::Message& m);
			void OnInfoRequest(const Comm::Message& m, double t);
			void AskPeer(const std::string& peer, double t);
			// Views of the fields or detections under prefix; keys (if given) gets each one's path or object id.
			std::vector<Environment::View> SharedViews(const std::string& prefix, bool own_only, bool changed_only, double t,
			                                           std::vector<std::string>* keys = nullptr);
			void PostViews(const std::string& code, const std::string& recipient, const std::vector<Environment::View>& views, double t);
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
			struct Tree {
				TaskTree tree;
				TreeState state;
				std::set<std::string> finished;  // nodes whose end was handled
				std::string last_finisher;       // who reported the latest DONE unit
				std::map<std::string, std::string> leaf_done_by;  // leaves inside complex units (spec 12 §2.2)
				explicit Tree(TaskTree t) : tree(std::move(t)), state(tree) {}
				Tree(const Tree&) = delete;
			};
			std::map<std::string, std::unique_ptr<Tree>> trees_;  // by root id
			std::map<std::string, Tree*> tree_of_;                 // every node id -> its tree
			PeerTable peers_;
			std::string assigned_;
			bool unit_started_ = false;  // a leaf of the assigned complex unit has run
			std::deque<std::pair<Task::TaskEvent, double>> robot_events_;
			std::deque<Robot::RaisedEvent> raised_;
			double now_ = 0.0;
			double last_state_ = -1e9, last_profile_ = -1e9;
			std::size_t max_payload_ = 0;
			std::map<std::string, double> sync_next_;   // by prefix: when it is due
			std::map<std::string, double> sync_sent_;   // by field path or detection id: stamp last sent
			std::set<std::string> asked_;               // peers sent an M_INFOREQ
			MrsStats stats_;
		};
	}
}
