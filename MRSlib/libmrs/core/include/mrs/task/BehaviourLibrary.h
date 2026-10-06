#pragma once
// The behaviour library (spec 03 §7): maps condition codes to the behaviour tasks that
// make the condition true.
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "mrs/task/Task.h"
#include "mrs/task/TaskFactory.h"

namespace MRS {
	namespace Task {
		struct BehaviourEntry {
			std::string name;       // "flyto.layered"
			std::string fulfils;    // a condition code, "C_P3"
			std::string qualifier;  // the predicate name for C_?, otherwise "any"
			double priority = 0.0;
			std::shared_ptr<const Task> behaviour;  // normally a T_B
		};

		class BehaviourLibrary {
		public:
			void Add(BehaviourEntry entry);
			// Adds every B_E record of a .mrsb text. Throws TaskLoadError.
			void Populate(const std::string& text, const TaskFactory& factory);
			void PopulateFromFile(const std::string& path, const TaskFactory& factory);

			// The entry for an unmet condition: same code (and, for C_?, the predicate with the
			// wanted value T), static requirements met when a profile is given, highest
			// priority, and the first in file order on a tie. nullptr when there is none.
			// Entries named in `exclude` (already tried for this condition) are skipped.
			const BehaviourEntry* Find(const Condition& unmet, const CapabilityProfile* profile = nullptr,
			                           const std::set<std::string>& exclude = {}) const;

			const std::vector<BehaviourEntry>& Entries() const { return entries_; }

		private:
			std::vector<BehaviourEntry> entries_;
		};
	}
}
