#pragma once
// Port assignment from a port map, explicit addresses or a scan (spec 04 §6.2-6.4).
// A port is never picked because it is free: same-type ports are not interchangeable.
#include <map>
#include <string>
#include <string_view>
#include <tuple>
#include <utility>
#include <vector>

#include "mrs/port/Port.h"
#include "mrs/protocol/Record.h"

namespace MRS {
	namespace Port {
		// The P_A records of a .mrsp file.
		struct PortMap {
			std::vector<PortAssignment> entries;

			// Throws BuildError on a record that is not P_A.
			static PortMap FromDocument(const Protocol::Document& doc);
			// Parses .mrsp text. Throws BuildError on a parse error.
			static PortMap Parse(std::string_view text);
			const PortAssignment* Find(const std::string& node, const std::string& requirement) const;
		};

		// The requirements of one node, in tree order.
		struct NodePorts {
			NodePorts() = default;
			NodePorts(std::string n, std::vector<PortRequirement> r, std::vector<std::string> p = {})
			    : node(std::move(n)), requirements(std::move(r)), providers(std::move(p)) {}

			std::string node;
			std::vector<PortRequirement> requirements;
			// The provider joints above the node, nearest first; the platform is implied after them (spec 16 §4.2).
			std::vector<std::string> providers;
		};

		struct UnresolvedPort {
			std::string node;
			std::string requirement;
			std::string reason;
		};

		struct PortReport {
			std::vector<PortAssignment> assigned;
			std::vector<UnresolvedPort> unresolved;
		};

		class PortManager {
		public:
			// The platform is the root provider, named `platform`.
			explicit PortManager(IPlatform& platform);

			// A provider joint that is ready to scan and open ports (spec 16 §4.1).
			void AddProvider(const std::string& name, IPortProvider& provider);
			bool HasProvider(const std::string& name) const { return providers_.count(name) != 0; }

			// Assigns every requirement (spec 04 §6.3), as a fresh assignment. Throws BuildError for a port map entry that
			// matches no requirement or has another type, and for an exclusive port given twice.
			PortReport Assign(const std::vector<NodePorts>& nodes, const PortMap* map = nullptr);

			// The two halves of Assign, for a robot that adds providers as it goes: the map check against every
			// requirement, then the requirements of one node, appended to `report`. Each requirement goes to the
			// nearest provider that offers its type (spec 16 §4.2). Throws BuildError when none does, or when a
			// port map entry's `via` names another provider.
			void CheckMap(const std::vector<NodePorts>& nodes, const PortMap* map) const;
			void AssignNode(const NodePorts& node, const PortMap* map, PortReport& report);

			// Opens one assigned port on its provider; null if it does not open.
			std::unique_ptr<Port> Open(const PortAssignment& assignment);

			// The scan of one provider, each port tagged with the provider's name (spec 16 §4.3). Cached.
			const std::vector<PortInfo>& Scan(const std::string& provider);

			// First-time setup (spec 04 §6.4): the port map of an assignment, as a .mrsp document. Entries not on
			// the platform carry `via` (spec 16 §4.2).
			static Protocol::Document WritePortMap(const PortReport& report);

		private:
			struct Use {
				std::string where;
				bool exclusive;
			};
			std::map<std::string, IPortProvider*> providers_;
			std::map<std::string, std::vector<PortInfo>> scans_;
			std::map<std::tuple<std::string, PortType, std::string>, std::vector<Use>> uses_;
		};
	}
}
