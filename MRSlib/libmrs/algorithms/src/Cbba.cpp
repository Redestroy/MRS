#include "mrs/algorithms/Cbba.h"

#include <algorithm>
#include <cmath>

namespace MRS {
	namespace Algorithms {
		using Protocol::Field;
		using Protocol::FieldType;

		namespace {
			constexpr double kLocked = 1e6;  // the bid of a started task: nobody outbids it
			constexpr double kEps = 1e-9;
			const char* kNone = "0";         // M_BID winner "0": no winner

			std::optional<std::array<double, 3>> Here(const Environment::Worldview& w, double t) {
				auto x = w.Scalar("pose.enu.x", t), y = w.Scalar("pose.enu.y", t), z = w.Scalar("pose.enu.z", t);
				if (!x || !y || !z) return std::nullopt;
				return std::array<double, 3>{*x, *y, *z};
			}
		}

		bool CbbaAllocator::Beats(double y, const std::string& z, const Bid& current) {
			if (z.empty()) return false;
			if (current.z.empty()) return y > 0.0;
			if (y > current.y + kEps) return true;
			return std::fabs(y - current.y) <= kEps && RobotNumber(z) < RobotNumber(current.z);
		}

		bool CbbaAllocator::Alive(const std::string& robot, double t) const {
			if (robot.empty()) return false;
			if (robot == ctx_.self || !ctx_.peers) return true;
			for (const Peer* p : ctx_.peers->Alive(t, ctx_.peer_timeout))
				if (p->name == robot) return true;
			return false;
		}

		void CbbaAllocator::Set(const std::string& task, const Bid& b) {
			table_[task] = b;
			dirty_.insert(task);
		}

		void CbbaAllocator::OnTaskReceived(const std::string&, double) { rebuild_ = true; }

		void CbbaAllocator::OnTaskFinished(const std::string& task, PoolState, double) {
			table_.erase(task);
			dirty_.erase(task);
			bundle_.erase(std::remove(bundle_.begin(), bundle_.end(), task), bundle_.end());
			path_.erase(std::remove(path_.begin(), path_.end(), task), path_.end());
			if (locked_ == task) locked_.clear();
			rebuild_ = true;
		}

		double CbbaAllocator::Score(const std::vector<std::string>& path, const std::array<double, 3>& here, double layer) const {
			double t = 0.0, s = 0.0;
			std::array<double, 3> pos = here;
			for (const auto& id : path) {
				const PoolEntry* e = ctx_.pool->Find(id);
				if (!e) continue;
				const TaskGeometry g = GeometryOf(e->task->ToRecord());
				if (g.has_target) {
					t += c_.travel.Time(pos, g.target, layer);
					pos = g.target;
				}
				t += c_.travel.settle + g.hold;
				s += e->task->Priority() * std::exp(-t / c_.horizon);
			}
			return s;
		}

		void CbbaAllocator::Build(const Environment::Worldview& w, double t) {
			rebuild_ = false;
			const auto here = Here(w, t);
			if (!here) return;
			const double layer = w.Scalar("layer.alt", t).value_or(c_.travel.cruise_alt);
			// Phase 1 (bundle construction): add the task of the highest marginal score that this
			// robot would win, at its best place in the path, until the bundle is full.
			const std::size_t first = locked_.empty() ? 0 : 1;  // the started task stays first
			while (static_cast<int>(bundle_.size()) < c_.bundle) {
				const double base = Score(path_, *here, layer);
				std::string best;
				double best_c = 0.0;
				std::size_t best_at = 0;
				for (const PoolEntry* e : ctx_.pool->Entries()) {
					if (!Eligible(*e, t)) continue;
					if (std::find(bundle_.begin(), bundle_.end(), e->id) != bundle_.end()) continue;
					Bid cur = table_.count(e->id) ? table_.at(e->id) : Bid{};
					if (!Alive(cur.z, t)) cur = {};
					for (std::size_t at = first; at <= path_.size(); ++at) {
						auto p = path_;
						p.insert(p.begin() + static_cast<long>(at), e->id);
						const double c = Score(p, *here, layer) - base;
						if (!Beats(c, ctx_.self, cur)) continue;
						if (best.empty() || c > best_c) {
							best = e->id;
							best_c = c;
							best_at = at;
						}
					}
				}
				if (best.empty()) break;
				bundle_.push_back(best);
				path_.insert(path_.begin() + static_cast<long>(best_at), best);
				Set(best, {best_c, ctx_.self, t});
			}
		}

		void CbbaAllocator::Release(std::size_t from, double t) {
			for (std::size_t k = from; k < bundle_.size(); ++k) {
				const std::string& id = bundle_[k];
				path_.erase(std::remove(path_.begin(), path_.end(), id), path_.end());
				// Tasks after the lost one were bid on assuming it: give them up too.
				if (k > from && table_.count(id) && table_[id].z == ctx_.self) Set(id, {0.0, {}, t});
				if (locked_ == id) locked_.clear();
			}
			bundle_.resize(from);
			rebuild_ = true;
		}

		void CbbaAllocator::Outbid(double t) {
			for (std::size_t k = 0; k < bundle_.size(); ++k) {
				auto it = table_.find(bundle_[k]);
				if (it == table_.end() || it->second.z != ctx_.self) {
					Release(k, t);
					return;
				}
			}
		}

		// Phase 2 (consensus), with the bid time as the freshness of a winner's own information.
		void CbbaAllocator::Merge(const std::string& sender, const std::string& task, const Bid& in, double t) {
			const PoolEntry* e = ctx_.pool->Find(task);
			if (e && Finished(e->state)) return;
			Bid cur = table_.count(task) ? table_.at(task) : Bid{};
			const bool cur_alive = Alive(cur.z, t);
			if (in.z == ctx_.self) return;  // only this robot speaks for its own bids
			if (in.z.empty()) {
				// The sender gave the task up, or heard that its winner did.
				if (cur.z == sender && in.time > cur.time) Set(task, {0.0, {}, in.time});
				return;
			}
			if (cur.z == in.z) {
				if (in.time > cur.time) Set(task, in);  // newer word from the same winner
				return;
			}
			if (!cur_alive || Beats(in.y, in.z, cur)) Set(task, in);
			(void)t;
		}

		void CbbaAllocator::OnMessage(const Comm::Message& m, double t) {
			if (m.code != "M_BID" || m.sender == ctx_.self || m.SlotCount() < 1) return;
			const auto n = static_cast<std::size_t>(m.Slot(0).i);
			for (std::size_t k = 0; k < n && 4 * k + 4 < m.SlotCount() + 0; ++k) {
				const std::string winner = m.Slot(4 * k + 3).s;
				Merge(m.sender, m.Slot(4 * k + 1).s, Bid{m.Slot(4 * k + 2).n, winner == kNone ? std::string() : winner, m.Slot(4 * k + 4).n}, t);
			}
			Outbid(t);
		}

		void CbbaAllocator::Send(double t) {
			if (!ctx_.send) return;
			const bool full = t - last_full_ >= c_.full_period;
			std::vector<std::string> ids;
			if (full) {
				for (const auto& [id, b] : table_) ids.push_back(id);
			} else {
				ids.assign(dirty_.begin(), dirty_.end());
			}
			if (ids.empty()) return;
			// Pace M_BID to the robot's share of a limited channel (spec 13 §3).
			double period = c_.bid_period;
			const double budget = ctx_.budget_bps ? ctx_.budget_bps(t) : 0.0;
			if (budget > 0.0) period = std::max(period, (80.0 + 40.0 * static_cast<double>(ids.size())) * 8.0 / (c_.budget_share * budget));
			if (t - last_sent_ < period) return;
			std::vector<Field> slots{Field::MakeInt(static_cast<std::int64_t>(ids.size()))};
			for (const auto& id : ids) {
				const Bid& b = table_.at(id);
				slots.push_back(Field::MakeText(FieldType::TaskId, id));
				slots.push_back(Field::MakeNum(b.y));
				slots.push_back(Field::MakeText(FieldType::Id, b.z.empty() ? kNone : b.z));
				slots.push_back(Field::MakeNum(b.time));
			}
			ctx_.send("M_BID", slots);
			++bids_sent_;
			last_sent_ = t;
			if (full) last_full_ = t;
			dirty_.clear();
		}

		Decision CbbaAllocator::Select(const Environment::Worldview& w, const CurrentTask& current, double t) {
			now_ = t;
			// A started task is held: its bid rises so that no robot takes it over.
			if (current.started && !path_.empty() && path_.front() == current.id && locked_ != current.id) {
				locked_ = current.id;
				Set(current.id, {kLocked, ctx_.self, t});
			}
			// Drop entries whose task ended while we did not hear of it.
			for (auto it = table_.begin(); it != table_.end();) {
				const PoolEntry* e = ctx_.pool->Find(it->first);
				if (e && Finished(e->state)) {
					dirty_.erase(it->first);
					it = table_.erase(it);
				} else {
					++it;
				}
			}
			Outbid(t);
			if (rebuild_ || static_cast<int>(bundle_.size()) < c_.bundle) Build(w, t);
			Send(t);
			for (const auto& id : path_) {
				const PoolEntry* e = ctx_.pool->Find(id);
				if (!e || !Eligible(*e, t)) continue;
				return {id == current.id ? Decision::Kind::KEEP : Decision::Kind::SWITCH, id};
			}
			if (current.started && !current.id.empty()) return {Decision::Kind::KEEP, current.id};
			return {Decision::Kind::IDLE, {}};
		}
	}
}
