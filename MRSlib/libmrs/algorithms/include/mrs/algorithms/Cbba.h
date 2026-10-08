#pragma once
// CBBA, the consensus-based bundle algorithm (Choi, Brunet and How, 2009; plan §8.5, spec 13 §2):
// each robot builds a bundle of up to `bundle` tasks by marginal score along its path, and the
// robots agree on one winner per task by exchanging winning bids (M_BID).
#include <map>
#include <set>
#include <string>
#include <vector>

#include "mrs/algorithms/Allocator.h"
#include "mrs/algorithms/Planner.h"

namespace MRS {
	namespace Algorithms {
		struct CbbaConfig {
			int bundle = 3;              // tasks per robot bundle (L_t)
			double horizon = 100.0;      // s: score = priority × exp(−arrival / horizon)
			double bid_period = 0.5;     // s: the shortest gap between two M_BID
			double full_period = 5.0;    // s: the whole table is sent this often, changes in between
			double budget_share = 0.5;   // of the robot's channel share for M_BID (spec 13 §3)
			TravelModel travel;          // arrival times along the path
		};

		class CbbaAllocator : public IAllocator {
		public:
			explicit CbbaAllocator(CbbaConfig c = {}) : c_(c) {}
			void OnTaskReceived(const std::string& task, double t) override;
			void OnMessage(const Comm::Message& m, double t) override;
			void OnTaskFinished(const std::string& task, PoolState s, double t) override;
			Decision Select(const Environment::Worldview& w, const CurrentTask& current, double t) override;
			AllocatorInfo Info() const override { return {"CBBA", true, true}; }

			struct Bid {
				double y = 0.0;      // the winning bid
				std::string z;       // the winner, empty for none
				double time = 0.0;   // when the winner made it
			};
			const std::map<std::string, Bid>& Table() const { return table_; }
			const std::vector<std::string>& Path() const { return path_; }
			const std::vector<std::string>& Bundle() const { return bundle_; }
			long BidsSent() const { return bids_sent_; }
			// Whether bid (y, z) beats the current entry: higher, or equal from a lower robot number.
			static bool Beats(double y, const std::string& z, const Bid& current);

		private:
			// The path's score from here: Σ priority × exp(−arrival / horizon).
			double Score(const std::vector<std::string>& path, const std::array<double, 3>& here, double layer) const;
			void Build(const Environment::Worldview& w, double t);
			void Release(std::size_t from, double t);  // drops bundle_[from..] and what follows it
			void Merge(const std::string& sender, const std::string& task, const Bid& in, double t);
			void Outbid(double t);
			void Send(double t);
			void Set(const std::string& task, const Bid& b);
			bool Alive(const std::string& robot, double t) const;

			CbbaConfig c_;
			std::map<std::string, Bid> table_;
			std::vector<std::string> bundle_, path_;
			std::set<std::string> dirty_;
			std::string locked_;   // the started task: its bid is held at kLocked
			bool rebuild_ = true;
			double last_sent_ = -1e9, last_full_ = -1e9;
			long bids_sent_ = 0;
			double now_ = 0.0;
		};
	}
}
