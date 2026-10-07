#pragma once
// What a robot knows about the team's work (plan §8.1, spec 09 §3): the task pool and the
// peer table.
#include <array>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "mrs/task/Task.h"

namespace MRS {
	namespace Algorithms {
		// Pool states (spec 06 §8, the J_T pool-state slot).
		enum class PoolState { AVAILABLE, BLOCKED, CLAIMED, ACTIVE, DONE, FAILED, DUMPED_SELF, IMPOSSIBLE, CANCELLED };
		const char* PoolStateName(PoolState s);
		bool Finished(PoolState s);  // DONE, FAILED, IMPOSSIBLE, CANCELLED

		struct Claim {
			std::string owner;     // robot name
			double expiry = 0.0;   // mission time
			double score = 0.0;
		};

		struct PoolEntry {
			std::string id;
			std::shared_ptr<const Task::Task> task;  // the task as received
			PoolState state = PoolState::AVAILABLE;
			double arrival = 0.0;                     // when this robot first heard of it
			std::map<std::string, Claim> claims;      // live claims by robot, from M_CLAIM
			std::set<std::string> static_dumps;       // robots that can never do it (incl. this one)
			double retry_after = 0.0;                 // dynamic dump cooldown on this robot
			std::string done_by;                      // who sent M_DONE
		};

		class TaskPool {
		public:
			// Adds a task; a known id changes nothing. Returns the entry when it is new.
			PoolEntry* Add(std::shared_ptr<const Task::Task> task, double t);
			PoolEntry* Find(const std::string& id);
			const PoolEntry* Find(const std::string& id) const;
			// Entries in arrival order.
			std::vector<PoolEntry*> Entries();
			std::vector<const PoolEntry*> Entries() const;
			std::size_t Size() const { return order_.size(); }
			// Every task has finished.
			bool AllFinished() const;

			// Drops claims whose expiry has passed.
			void ExpireClaims(double t);
			// The strongest live claim of another robot: the higher score, then the lower robot id.
			static std::optional<Claim> BestClaim(const PoolEntry& e, const std::string& except = {});
			// Whether claim a beats claim b (spec 06 §4.1).
			static bool Beats(const Claim& a, const Claim& b);

		private:
			std::map<std::string, PoolEntry> entries_;
			std::vector<std::string> order_;
		};

		struct Peer {
			std::string name;           // "r3"
			double last_heard = -1.0;
			std::array<double, 3> pose{};
			std::array<double, 3> vel{};
			double battery = -1.0;      // remaining fraction
			std::string task;           // current task id, "0" for none
			std::string robot_type;
			std::set<std::string> roles;
			std::set<std::string> actions;  // from M_PROFILE
			bool has_profile = false;
		};

		class PeerTable {
		public:
			Peer& Touch(const std::string& name, double t);
			const Peer* Find(const std::string& name) const;
			// Peers heard within timeout of t.
			std::vector<const Peer*> Alive(double t, double timeout) const;
			const std::map<std::string, Peer>& All() const { return peers_; }

		private:
			std::map<std::string, Peer> peers_;
		};

		// The robot number of a name "r<N>", or a large number for other names (spec 06 §1).
		long RobotNumber(const std::string& name);
	}
}
