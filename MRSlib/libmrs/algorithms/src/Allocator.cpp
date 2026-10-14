#include "mrs/algorithms/Allocator.h"

#include <algorithm>
#include <cmath>

namespace MRS {
	namespace Algorithms {
		namespace {
			std::optional<std::array<double, 3>> Here(const Environment::Worldview& w, double t) {
				auto x = w.Scalar("pose.enu.x", t), y = w.Scalar("pose.enu.y", t), z = w.Scalar("pose.enu.z", t);
				if (!x || !y || !z) return std::nullopt;
				return std::array<double, 3>{*x, *y, *z};
			}
		}

		std::optional<double> SpatialSizeEstimator::Travel(const Task::Task& task, const Environment::Worldview& w, double t) {
			const auto here = Here(w, t);
			const auto targets = Robot::ResourceManager::Targets(task);
			if (!here || targets.empty()) return std::nullopt;
			const auto& g = targets.front();
			const double across = std::hypot(g[0] - (*here)[0], g[1] - (*here)[1]);
			const auto layer = w.Raw("layer.alt");
			// The layered fly-to goes straight once within its switch radius (spec 02 §8, 3 m in
			// uav_behaviours.mrsb), so a robot over its target does not count the climb to its layer.
			constexpr double kSwitchRadius = 3.0;
			if (across > kSwitchRadius && layer && layer->valid && std::holds_alternative<double>(layer->value)) {
				const double alt = std::get<double>(layer->value);
				return std::fabs(alt - (*here)[2]) + across + std::fabs(alt - g[2]);
			}
			return std::hypot(across, g[2] - (*here)[2]);
		}

		double SpatialSizeEstimator::Estimate(const PoolEntry& e, const Environment::Worldview& w, double t, const AllocatorContext& ctx) const {
			const double waited = std::max(0.0, t - e.arrival);
			const double ds = Travel(*e.task, w, t).value_or(0.0);
			double energy_fraction = 0.0;
			if (ctx.resources) {
				const auto cost = ctx.resources->TaskCost(*e.task, w, t);
				const auto remaining = w.Scalar("battery.energy_wh", t);
				if (cost && remaining && *remaining > 0.0) energy_fraction = std::min(1.0, *cost / *remaining);
			}
			double penalty = 0.0;
			bool pursued = false;
			const auto targets = Robot::ResourceManager::Targets(*e.task);
			const auto here = Here(w, t);
			if (ctx.peers && !targets.empty() && here) {
				const auto& g = targets.front();
				const double mine = std::hypot(g[0] - (*here)[0], g[1] - (*here)[1]);
				auto closer = [&](const Peer& p) { return std::hypot(g[0] - p.pose[0], g[1] - p.pose[1]) < mine; };
				for (const auto& [owner, claim] : e.claims) {
					if (owner == ctx.self) continue;
					const Peer* p = ctx.peers->Find(owner);
					if (p && closer(*p)) penalty = 1.0;
				}
				for (const Peer* p : ctx.peers->Alive(t, ctx.peer_timeout))
					if (p->name != ctx.self && p->task == e.id && closer(*p)) pursued = true;
			}
			double s = c_.a1 * waited + c_.a2 / (ds + c_.epsilon) - c_.a3 * energy_fraction - c_.a4 * penalty;
			if (pursued) s *= c_.pursuit;
			return std::max(c_.floor, s);
		}

		bool IAllocator::Eligible(const PoolEntry& e, double t) const {
			if (Finished(e.state) || e.state == PoolState::DUMPED_SELF || e.state == PoolState::BLOCKED) return false;
			if (e.retry_after > t) return false;
			return !ctx_.runnable || ctx_.runnable(e, t);
		}

		double IAllocator::Priority(const PoolEntry& e, const Environment::Worldview& w, double t) const {
			const double size = ctx_.size ? ctx_.size->Estimate(e, w, t, ctx_) : 1.0;
			return e.task->Priority() * size;  // the base priority once (the 2021 code applied it twice)
		}

		bool RtaAllocator::Outclaimed(const PoolEntry& e, double priority) const {
			if (!c_.exclusive) return false;
			const auto best = TaskPool::BestClaim(e, ctx_.self);
			return best && TaskPool::Beats(*best, Claim{ctx_.self, 0.0, priority});
		}

		void RtaAllocator::ClaimFor(const std::string& task, double priority, double t) {
			if (!c_.exclusive) return;
			if (!claimed_.empty() && claimed_ != task && ctx_.release) ctx_.release(claimed_);
			claimed_ = task;
			claim_score_ = priority;
			claim_expiry_ = t + c_.claim_ttl;
			if (ctx_.claim) ctx_.claim(task, claim_expiry_, priority);
		}

		Decision RtaAllocator::Select(const Environment::Worldview& w, const CurrentTask& current, double t) {
			const PoolEntry* best = nullptr;
			double best_p = 0.0;
			for (const PoolEntry* e : ctx_.pool->Entries()) {
				if (!Eligible(*e, t) || !Allowed(*e)) continue;
				const double p = Score(*e, w, t);
				if (Outclaimed(*e, p)) continue;
				if (!best || p > best_p) {
					best = e;
					best_p = p;
				}
			}

			const PoolEntry* cur = current.id.empty() ? nullptr : ctx_.pool->Find(current.id);
			const bool cur_ok = cur && Eligible(*cur, t) && (Allowed(*cur) || current.started);
			const double cur_p = cur_ok ? Score(*cur, w, t) : 0.0;
			// The stronger claim keeps a task, until its actions run: a started task stays with its
			// robot, whose claim does not weaken as it works away from the task's first target
			// (spec 12 §3.2; a complex unit's robot moves off it from its second leaf on).
			const bool cur_lost = cur_ok && !current.started && Outclaimed(*cur, cur_p);

			if (cur_ok && !cur_lost) {
				const bool better = best && best != cur && best_p > cur_p * (1.0 + c_.switch_margin);
				if (current.started || !better) {
					const double claim_p = current.started && claimed_ == cur->id ? std::max(cur_p, claim_score_) : cur_p;
					if (c_.exclusive && (claimed_ != cur->id || claim_expiry_ - t < c_.claim_ttl / 2)) ClaimFor(cur->id, claim_p, t);
					return {Decision::Kind::KEEP, cur->id};
				}
			}
			if (!best) {
				if (c_.exclusive && !claimed_.empty()) {
					if (ctx_.release) ctx_.release(claimed_);
					claimed_.clear();
				}
				return {Decision::Kind::IDLE, {}};
			}
			ClaimFor(best->id, best_p, t);
			return {Decision::Kind::SWITCH, best->id};
		}

		void RtaAllocator::OnTaskFinished(const std::string& task, PoolState, double) {
			if (task == claimed_) claimed_.clear();  // a finished task needs no release
		}
	
		// --- MRS-STA (spec 12 §3) -----------------------------------------------------------------

		StaAllocator::StaAllocator(StaConfig c) : RtaAllocator([&] {
			RtaConfig r = c.rta;
			r.exclusive = true;
			return r;
		}()), s_(c) {}

		namespace {
			// The dot-separated parts of an id below the root.
			std::vector<std::string> Path(const std::string& root, const std::string& id) {
				std::vector<std::string> out;
				if (id.size() <= root.size() + 1 || id.compare(0, root.size(), root) != 0) return out;
				std::string part;
				for (std::size_t i = root.size() + 1; i <= id.size(); ++i) {
					if (i == id.size() || id[i] == '.') {
						out.push_back(part);
						part.clear();
					} else {
						part += id[i];
					}
				}
				return out;
			}
		}

		double StaAllocator::Kinship(const TaskTree& tree, const std::string& unit, const std::string& other) {
			const auto a = Path(tree.root, unit), b = Path(tree.root, other);
			if (a.empty() || b.empty()) return 0.0;
			std::size_t common = 0;
			while (common < a.size() && common < b.size() && a[common] == b[common]) ++common;
			return static_cast<double>(common) / static_cast<double>(a.size());
		}

		double StaAllocator::Score(const PoolEntry& e, const Environment::Worldview& w, double t) const {
			double p = Priority(e, w, t);
			const TaskTree* tree = ctx_.tree_of ? ctx_.tree_of(e.id) : nullptr;
			if (!tree) return p;
			if (std::find(stack_.begin(), stack_.end(), tree->root) != stack_.end()) p *= 1.0 + s_.stack_bonus;
			if (!last_.empty() && tree->Has(last_)) p *= 1.0 + s_.relative_bonus * Kinship(*tree, e.id, last_);
			return p;
		}

		Decision StaAllocator::Select(const Environment::Worldview& w, const CurrentTask& current, double t) {
			// A tree leaves the stack once none of its units is left for anyone.
			stack_.erase(std::remove_if(stack_.begin(), stack_.end(),
			                            [this](const std::string& root) {
				                            const TaskTree* tree = ctx_.tree_of ? ctx_.tree_of(root) : nullptr;
				                            if (!tree) return true;
				                            for (const auto& u : tree->units)
					                            if (const PoolEntry* e = ctx_.pool->Find(u); e && !Finished(e->state)) return false;
				                            return true;
			                            }),
			             stack_.end());
			const Decision d = RtaAllocator::Select(w, current, t);
			if (d.kind != Decision::Kind::IDLE && ctx_.tree_of)
				if (const TaskTree* tree = ctx_.tree_of(d.task)) {
					stack_.erase(std::remove(stack_.begin(), stack_.end(), tree->root), stack_.end());
					stack_.push_back(tree->root);
				}
			return d;
		}

		void StaAllocator::OnTaskFinished(const std::string& task, PoolState s, double t) {
			RtaAllocator::OnTaskFinished(task, s, t);
			if (s != PoolState::DONE) return;
			if (const PoolEntry* e = ctx_.pool->Find(task); e && e->done_by == ctx_.self) last_ = task;
		}
	}
}
