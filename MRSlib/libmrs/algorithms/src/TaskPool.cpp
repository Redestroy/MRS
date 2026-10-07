#include "mrs/algorithms/TaskPool.h"

#include <algorithm>
#include <cstdlib>
#include <limits>

namespace MRS {
	namespace Algorithms {
		const char* PoolStateName(PoolState s) {
			switch (s) {
			case PoolState::AVAILABLE: return "AVAILABLE";
			case PoolState::BLOCKED: return "BLOCKED";
			case PoolState::CLAIMED: return "CLAIMED";
			case PoolState::ACTIVE: return "ACTIVE";
			case PoolState::DONE: return "DONE";
			case PoolState::FAILED: return "FAILED";
			case PoolState::DUMPED_SELF: return "DUMPED_SELF";
			case PoolState::IMPOSSIBLE: return "IMPOSSIBLE";
			case PoolState::CANCELLED: return "CANCELLED";
			}
			return "?";
		}

		bool Finished(PoolState s) {
			return s == PoolState::DONE || s == PoolState::FAILED || s == PoolState::IMPOSSIBLE || s == PoolState::CANCELLED;
		}

		long RobotNumber(const std::string& name) {
			if (name.size() > 1 && name[0] == 'r' && std::all_of(name.begin() + 1, name.end(), [](char c) { return c >= '0' && c <= '9'; }))
				return std::strtol(name.c_str() + 1, nullptr, 10);
			return std::numeric_limits<long>::max();
		}

		PoolEntry* TaskPool::Add(std::shared_ptr<const Task::Task> task, double t) {
			const std::string id = task->Id();
			if (entries_.count(id)) return nullptr;
			PoolEntry e;
			e.id = id;
			e.task = std::move(task);
			e.arrival = t;
			order_.push_back(id);
			return &(entries_[id] = std::move(e));
		}

		PoolEntry* TaskPool::Find(const std::string& id) {
			auto it = entries_.find(id);
			return it == entries_.end() ? nullptr : &it->second;
		}

		const PoolEntry* TaskPool::Find(const std::string& id) const {
			auto it = entries_.find(id);
			return it == entries_.end() ? nullptr : &it->second;
		}

		std::vector<PoolEntry*> TaskPool::Entries() {
			std::vector<PoolEntry*> out;
			for (const auto& id : order_) out.push_back(&entries_.at(id));
			return out;
		}

		std::vector<const PoolEntry*> TaskPool::Entries() const {
			std::vector<const PoolEntry*> out;
			for (const auto& id : order_) out.push_back(&entries_.at(id));
			return out;
		}

		bool TaskPool::AllFinished() const {
			for (const auto& [id, e] : entries_)
				if (!Finished(e.state)) return false;
			return true;
		}

		void TaskPool::ExpireClaims(double t) {
			for (auto& [id, e] : entries_)
				for (auto it = e.claims.begin(); it != e.claims.end();) it = it->second.expiry < t ? e.claims.erase(it) : std::next(it);
		}

		bool TaskPool::Beats(const Claim& a, const Claim& b) {
			if (a.score != b.score) return a.score > b.score;
			return RobotNumber(a.owner) < RobotNumber(b.owner) || (RobotNumber(a.owner) == RobotNumber(b.owner) && a.owner < b.owner);
		}

		std::optional<Claim> TaskPool::BestClaim(const PoolEntry& e, const std::string& except) {
			std::optional<Claim> best;
			for (const auto& [owner, c] : e.claims) {
				if (owner == except) continue;
				if (!best || Beats(c, *best)) best = c;
			}
			return best;
		}

		Peer& PeerTable::Touch(const std::string& name, double t) {
			Peer& p = peers_[name];
			p.name = name;
			p.last_heard = std::max(p.last_heard, t);
			return p;
		}

		const Peer* PeerTable::Find(const std::string& name) const {
			auto it = peers_.find(name);
			return it == peers_.end() ? nullptr : &it->second;
		}

		std::vector<const Peer*> PeerTable::Alive(double t, double timeout) const {
			std::vector<const Peer*> out;
			for (const auto& [name, p] : peers_)
				if (p.last_heard >= 0.0 && t - p.last_heard <= timeout) out.push_back(&p);
			return out;
		}
	}
}
