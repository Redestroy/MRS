#include "mrs/algorithms/Ldta2.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace MRS {
	namespace Algorithms {
		Ldta2Allocator::Ldta2Allocator(Ldta2Config c) : RtaAllocator([&] {
			RtaConfig r = c.rta;
			r.exclusive = true;
			return r;
		}()) {}

		std::string Ldta2Allocator::TypeOf(const std::string& task) const {
			if (task.empty() || task == "0") return {};
			if (ctx_.tree_of)
				if (const TaskTree* tree = ctx_.tree_of(task)) return tree->root;
			return "*";
		}

		std::map<std::string, int> Ldta2Allocator::Wanted(const std::map<std::string, int>& open, int robots) {
			std::map<std::string, int> c;
			int total = 0;
			for (const auto& [type, n] : open) total += n;
			if (total == 0 || robots <= 0) return c;
			std::vector<std::pair<double, std::string>> rest;
			int given = 0;
			for (const auto& [type, n] : open) {
				const double exact = static_cast<double>(robots) * n / total;
				c[type] = static_cast<int>(std::floor(exact));
				given += c[type];
				rest.push_back({exact - std::floor(exact), type});
			}
			std::stable_sort(rest.begin(), rest.end(), [](const auto& a, const auto& b) { return a.first > b.first; });
			for (std::size_t k = 0; given < robots && k < rest.size(); ++k, ++given) ++c[rest[k].second];
			// A type is never wanted by more robots than it has open tasks.
			for (auto& [type, n] : c) n = std::min(n, open.at(type));
			return c;
		}

		Decision Ldta2Allocator::Select(const Environment::Worldview& w, const CurrentTask& current, double t) {
			// Update: the open tasks per type, and the type every live robot works on (its M_STATE).
			std::map<std::string, int> open;
			for (const PoolEntry* e : ctx_.pool->Entries())
				if (!Finished(e->state) && e->state != PoolState::DUMPED_SELF) ++open[TypeOf(e->id)];
			std::map<std::string, std::string> on;  // robot -> type ("" idle)
			on[ctx_.self] = TypeOf(current.id);
			if (ctx_.peers)
				for (const Peer* p : ctx_.peers->Alive(t, ctx_.peer_timeout))
					if (p->name != ctx_.self) on[p->name] = open.count(TypeOf(p->task)) ? TypeOf(p->task) : std::string();
			const auto wanted = Wanted(open, static_cast<int>(on.size()));
			std::map<std::string, int> have;
			for (const auto& [robot, type] : on)
				if (!type.empty()) ++have[type];
			const auto over = [&](const std::string& type) {
				return type.empty() || !wanted.count(type) || have[type] > wanted.at(type);
			};

			const std::string mine = on[ctx_.self];
			type_.clear();
			if (!over(mine)) {
				type_ = mine;  // the type has no more robots than it needs: keep to it
			} else {
				// AdjustTask: the type that lacks the most robots (δ = max(0, C − C_A)).
				std::string best;
				int delta = 0;
				for (const auto& [type, c] : wanted) {
					const int d = std::max(0, c - have[type]);
					if (d > delta) {
						best = type;
						delta = d;
					}
				}
				// The robots that will move before this one: lower numbers on over-filled or no types.
				int before = 0;
				for (const auto& [robot, type] : on)
					if (robot != ctx_.self && RobotNumber(robot) < RobotNumber(ctx_.self) && over(type)) ++before;
				if (!best.empty() && delta > before) type_ = best;
				else if (!mine.empty() && open.count(mine)) type_ = mine;  // not our turn: stay
				// An idle robot whose turn has not come takes what it would take anyway.
			}
			Decision d = RtaAllocator::Select(w, current, t);
			if (d.kind == Decision::Kind::IDLE && !type_.empty()) {
				type_.clear();  // nothing it may take in its type: fall back to any task
				d = RtaAllocator::Select(w, current, t);
			}
			return d;
		}
	}
}
