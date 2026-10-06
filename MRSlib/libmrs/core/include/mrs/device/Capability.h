#pragma once
// Capabilities (K records, spec 04 §4.1) and views (V records, spec 05 §6).
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "mrs/protocol/Record.h"
#include "mrs/world/View.h"

namespace MRS {
	namespace Device {
		enum class CapabilityKind { Action, View, Message, Capacity };  // K_A, K_V, K_M, K_Q

		struct Capability {
			CapabilityKind kind = CapabilityKind::Action;
			std::string code;       // action code, view code, message mode or resource name
			double capacity = 0.0;  // K_Q only
			std::map<std::string, double> limits;
			std::string node;       // the node that provides it; set by the tree builder

			static Capability Act(std::string code, std::map<std::string, double> limits = {});
			static Capability ViewOf(std::string code, std::map<std::string, double> limits = {});
			static Capability Messages(std::string mode, std::map<std::string, double> limits = {});
			static Capability Store(std::string resource, double capacity, std::map<std::string, double> limits = {});

			// Same kind and code: what a K override in a definition file matches on.
			bool SameAs(const Capability& other) const { return kind == other.kind && code == other.code; }

			Protocol::Record ToRecord() const;
			static std::optional<Capability> FromRecord(const Protocol::Record& rec);
		};

		const char* CapabilityCode(CapabilityKind kind);  // "K_A", ...

		// Views are worldview input (spec 05 §6); devices produce them.
		using View = Environment::View;
	}
}
