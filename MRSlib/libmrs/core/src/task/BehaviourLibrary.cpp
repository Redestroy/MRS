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

		// A qualifier segment "_" matches any one dot-separated segment: "zone._.mine" matches
		// "zone.3.mine" and "zone.dest.mine" (spec 18 §4.3).
		bool QualifierMatches(const std::string& pattern, const std::string& name) {
			if (pattern == name) return true;
			if (pattern.find("._") == std::string::npos && pattern.rfind("_.", 0) != 0) return false;
			auto split = [](const std::string& s) {
				std::vector<std::string> out;
				std::size_t start = 0;
				while (true) {
					const std::size_t end = s.find('.', start);
					out.push_back(s.substr(start, end == std::string::npos ? std::string::npos : end - start));
					if (end == std::string::npos) return out;
					start = end + 1;
				}
			};
			const auto ps = split(pattern), ns = split(name);
			if (ps.size() != ns.size()) return false;
			for (std::size_t k = 0; k < ps.size(); ++k)
				if (ps[k] != "_" && ps[k] != ns[k]) return false;
			return true;
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
				if (predicate && !sync && !QualifierMatches(e.qualifier, unmet.Qualifier())) continue;
				if (profile && !e.behaviour->MeetsStaticRequirements(*profile)) continue;
				if (!best || e.priority > best->priority) best = &e;
			}
			return best;
		}
	}
}
