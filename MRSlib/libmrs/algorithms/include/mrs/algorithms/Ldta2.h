#pragma once
// LDTA², the local dynamic task allocation algorithm (de Mendonça, Nedjah and de Macedo Mourelle
// 2016, as described in JB's bachelor thesis §2.1.1; spec 13 §5): robots spread over task types
// in wanted proportions. A robot leaves its type only when the type has more robots than it
// needs, for the type that lacks the most, and robots leave in order of their number so that not
// all of them move at once.
#include <map>
#include <string>

#include "mrs/algorithms/Allocator.h"

namespace MRS {
	namespace Algorithms {
		struct Ldta2Config {
			RtaConfig rta{true};  // picks the task within a type (MRS-RTA-X)
		};

		class Ldta2Allocator : public RtaAllocator {
		public:
			explicit Ldta2Allocator(Ldta2Config c = {});
			Decision Select(const Environment::Worldview& w, const CurrentTask& current, double t) override;
			AllocatorInfo Info() const override { return {"LDTA2", true, true}; }

			// A task's type: its tree's root for a unit of a split tree, "*" for any other task.
			std::string TypeOf(const std::string& task) const;
			// Robots wanted per type, C = P × ρ with P ∝ the type's open tasks, rounded by largest
			// remainder so that the counts add up to ρ.
			static std::map<std::string, int> Wanted(const std::map<std::string, int>& open, int robots);
			const std::string& Type() const { return type_; }

		protected:
			bool Allowed(const PoolEntry& e) const override { return type_.empty() || TypeOf(e.id) == type_; }

		private:
			std::string type_;  // the type Select may pick from; empty: any
		};
	}
}
