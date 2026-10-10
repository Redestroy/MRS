#include "mrs/task/BehaviourLibrary.h"

#include <fstream>
#include <sstream>

#include "mrs/protocol/Parser.h"

namespace MRS {
	namespace Task {
		void BehaviourLibrary::Add(BehaviourEntry entry) { entries_.push_back(std::move(entry)); }

		void BehaviourLibrary::Populate(const std::string& text, const TaskFactory& factory) {
			auto parsed = Protocol::Parse(text);
			if (!parsed.Ok())
				throw TaskLoadError(std::string(Protocol::ErrorClassName(parsed.error->error_class)) + " at byte " +
				                    std::to_string(parsed.error->offset) + ": " + parsed.error->message);
			for (const auto& r : parsed.document.records) {
				if (r.code != "B_E") continue;
				BehaviourEntry e;
				e.name = r.fields[0].s;
				e.fulfils = r.fields[1].s;
				e.qualifier = r.fields[2].s;
				e.priority = r.fields[3].n;
				e.behaviour = factory.BuildTask(r.children.at(r.fields[4].ref));
				Add(std::move(e));
			}
		}

		void BehaviourLibrary::PopulateFromFile(const std::string& path, const TaskFactory& factory) {
			std::ifstream in(path, std::ios::binary);
			if (!in) throw TaskLoadError("cannot open " + path);
			std::stringstream buffer;
			buffer << in.rdbuf();
			Populate(buffer.str(), factory);
		}

		const BehaviourEntry* BehaviourLibrary::ByName(const std::string& name) const {
			for (const auto& e : entries_)
				if (e.name == name) return &e;
			return nullptr;
		}

		bool BehaviourLibrary::SetPriority(const std::string& name, double priority) {
			bool found = false;
			for (auto& e : entries_)
				if (e.name == name) {
					e.priority = priority;
					found = true;
				}
			return found;
		}

		const BehaviourEntry* BehaviourLibrary::Find(const Condition& unmet, const CapabilityProfile* profile,
		                                             const std::set<std::string>& exclude, bool sync_only) const {
			const bool predicate = unmet.Code() == "C_?";
			// A C_? that wants F has no behaviour in version 0.1 (spec 03 §7).
			if (predicate && !unmet.WantedValue()) return nullptr;
			const bool shared = !SharedTopics(unmet).empty();
			const BehaviourEntry* best = nullptr;
			for (const auto& e : entries_) {
				if (e.fulfils != unmet.Code() || exclude.count(e.name)) continue;
				const bool sync = e.qualifier == "sync";
				if (sync ? !shared : sync_only) continue;
				if (predicate && !sync && e.qualifier != unmet.Qualifier()) continue;
				if (profile && !e.behaviour->MeetsStaticRequirements(*profile)) continue;
				if (!best || e.priority > best->priority) best = &e;
			}
			return best;
		}
	}
}
