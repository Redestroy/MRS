#pragma once
// The allocator seam (plan §8.2, spec 09 §4), the priority estimate (plan §8.4), MRS-RTA and
// MRS-STA (spec 12 §3).
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "mrs/algorithms/TaskPool.h"
#include "mrs/algorithms/TaskTree.h"
#include "mrs/comm/Messenger.h"
#include "mrs/robot/Resources.h"
#include "mrs/world/Worldview.h"

namespace MRS {
	namespace Algorithms {
		struct AllocatorContext;

		// size_est(task, w, t) of Priority = base_priority × size_est (JB decision 2).
		class ISizeEstimator {
		public:
			virtual ~ISizeEstimator() = default;
			virtual double Estimate(const PoolEntry& entry, const Environment::Worldview& w, double t, const AllocatorContext& ctx) const = 0;
		};

		struct SpatialSizeConfig {
			double a1 = 0.001;    // per second waited
			double a2 = 10.0;     // m: weight of closeness
			double a3 = 0.5;      // weight of the energy fraction
			double a4 = 0.5;      // claim penalty
			double pursuit = 0.2; // factor when a closer live peer already pursues the task (1: off)
			double epsilon = 1.0; // m
			double floor = 1e-3;  // the estimate never drops below this
		};

		// size_est = a1·Δt + a2/(Δs3 + ε) − a3·energy_fraction − a4·claim_penalty (plan §8.4).
		//   Δt: time since this robot heard of the task
		//   Δs3: travel from here to the task's first target the layered way (up to layer.alt,
		//        across, down), or straight when the robot has no layer
		//   energy_fraction: task cost / remaining energy, 0 without a resource manager
		//   claim_penalty: 1 when a peer that claims the task is closer to it
		// The result is multiplied by `pursuit` when a live peer that is closer to the task reports
		// it as its current task in M_STATE (spec 09 §4.2): open mode then spreads the robots
		// instead of sending them all to the same task.
		class SpatialSizeEstimator : public ISizeEstimator {
		public:
			explicit SpatialSizeEstimator(SpatialSizeConfig c = {}) : c_(c) {}
			double Estimate(const PoolEntry& entry, const Environment::Worldview& w, double t, const AllocatorContext& ctx) const override;
			// The travel distance Δs3 from here, or nullopt without a position or a target.
			static std::optional<double> Travel(const Task::Task& task, const Environment::Worldview& w, double t);
			const SpatialSizeConfig& Config() const { return c_; }

		private:
			SpatialSizeConfig c_;
		};

		// What an allocator works with. The MRS layer fills it in.
		struct AllocatorContext {
			TaskPool* pool = nullptr;
			PeerTable* peers = nullptr;
			std::string self;                                   // "r3"
			const Task::CapabilityProfile* profile = nullptr;
			const ISizeEstimator* size = nullptr;
			const Robot::ResourceManager* resources = nullptr;  // may be null
			double peer_timeout = 10.0;
			// Messages the allocator may send.
			std::function<void(const std::string& task, double expiry, double score)> claim;
			std::function<void(const std::string& task)> release;
			// False when the robot cannot run the task now (a runtime condition is FALSE); may be empty.
			std::function<bool(const PoolEntry& entry, double t)> runnable;
			// The tree a unit belongs to, or nullptr (spec 12 §2); may be empty.
			std::function<const TaskTree*(const std::string& id)> tree_of;
		};

		struct Decision {
			enum class Kind { KEEP, SWITCH, IDLE };
			Kind kind = Kind::IDLE;
			std::string task;  // SWITCH: the task to pursue
		};

		struct AllocatorInfo {
			std::string name;
			bool exclusive = false;
			bool uses_comms = false;
		};

		// What the robot is doing, for Select.
		struct CurrentTask {
			std::string id;         // empty: none
			bool started = false;   // its actions are running (IN_PROGRESS): it is not switched lightly
		};

		class IAllocator {
		public:
			virtual ~IAllocator() = default;
			virtual void Bind(const AllocatorContext& ctx) { ctx_ = ctx; }
			virtual void OnTaskReceived(const std::string& task, double t) { (void)task, (void)t; }
			virtual void OnMessage(const Comm::Message& m, double t) { (void)m, (void)t; }
			virtual void OnTaskFinished(const std::string& task, PoolState s, double t) { (void)task, (void)s, (void)t; }
			virtual void OnCapabilityChanged(double t) { (void)t; }
			virtual Decision Select(const Environment::Worldview& w, const CurrentTask& current, double t) = 0;
			virtual AllocatorInfo Info() const = 0;

		protected:
			// A task this robot may pick now: not finished, not dumped by it, past its retry cooldown.
			bool Eligible(const PoolEntry& e, double t) const;
			// base_priority × size_est.
			double Priority(const PoolEntry& e, const Environment::Worldview& w, double t) const;
			AllocatorContext ctx_;
		};

		struct RtaConfig {
			bool exclusive = false;   // claims (spec 06 §4.1)
			double switch_margin = 0.25;  // switch only to a task at least this much (relative) better
			double claim_ttl = 10.0;      // s a claim lasts; renewed at half-life
		};

		// MRS-RTA (the 2021 paper): every robot picks the known task of highest priority and
		// works on it until it is done by anyone. Open mode lets several robots go for one task;
		// exclusive mode claims it, and the stronger claim keeps it.
		class RtaAllocator : public IAllocator {
		public:
			explicit RtaAllocator(RtaConfig c = {}) : c_(c) {}
			Decision Select(const Environment::Worldview& w, const CurrentTask& current, double t) override;
			void OnTaskFinished(const std::string& task, PoolState s, double t) override;
			AllocatorInfo Info() const override { return {c_.exclusive ? "MRS-RTA-X" : "MRS-RTA", c_.exclusive, true}; }

		protected:
			// What Select ranks tasks by: base_priority × size_est here.
			virtual double Score(const PoolEntry& e, const Environment::Worldview& w, double t) const { return Priority(e, w, t); }

		private:
			// Whether another robot's claim beats ours at this priority.
			bool Outclaimed(const PoolEntry& e, double priority) const;
			void ClaimFor(const std::string& task, double priority, double t);

			RtaConfig c_;
			std::string claimed_;       // the task we claim
			double claim_expiry_ = 0.0;
			double claim_score_ = 0.0;  // the score we last claimed with
		};
	
		struct StaConfig {
			RtaConfig rta{true};        // STA always claims (spec 12 §3.1)
			double stack_bonus = 0.5;    // × (1 + this) for a unit of a tree on the active-task stack
			double relative_bonus = 0.5; // × (1 + this × kinship) with the unit this robot finished last
		};

		// MRS-STA (plan §8.5, spec 12 §3): MRS-RTA-X over the units of split trees, with an
		// active-task stack. A robot that takes a unit of a tree pushes the tree's root; while the
		// tree has units left the root stays on the stack, and the robot prefers its units, most
		// of all those nearest in the tree to the unit it finished last (kinship: the share of the
		// unit's path below the root that the two have in common). Tasks outside trees rank as
		// in MRS-RTA-X.
		class StaAllocator : public RtaAllocator {
		public:
			explicit StaAllocator(StaConfig c = {});
			Decision Select(const Environment::Worldview& w, const CurrentTask& current, double t) override;
			void OnTaskFinished(const std::string& task, PoolState s, double t) override;
			AllocatorInfo Info() const override { return {"MRS-STA", true, true}; }
			// Tree roots, the latest on top.
			const std::vector<std::string>& Stack() const { return stack_; }
			// 0..1: how much of `unit`'s path below the root it shares with `other`.
			static double Kinship(const TaskTree& tree, const std::string& unit, const std::string& other);

		protected:
			double Score(const PoolEntry& e, const Environment::Worldview& w, double t) const override;

		private:
			StaConfig s_;
			std::vector<std::string> stack_;
			std::string last_;  // the unit this robot finished last
		};
	}
}
