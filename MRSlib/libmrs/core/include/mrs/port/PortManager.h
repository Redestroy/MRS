#pragma once
// Port assignment from a port map, explicit addresses or a scan (spec 04 §6.2-6.4).
// A port is never picked because it is free: same-type ports are not interchangeable.
#include <string>
#include <string_view>
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
			std::string node;
			std::vector<PortRequirement> requirements;
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
			explicit PortManager(IPlatform& platform) : platform_(platform) {}

			// Assigns every requirement (spec 04 §6.3). Throws BuildError for a port map entry that
			// matches no requirement or has another type, and for an exclusive port given twice.
			PortReport Assign(const std::vector<NodePorts>& nodes, const PortMap* map = nullptr);

			// Opens one assigned port on the platform; null if it does not open.
			std::unique_ptr<Port> Open(const PortAssignment& assignment) { return platform_.Open(assignment); }

			// First-time setup (spec 04 §6.4): the port map of an assignment, as a .mrsp document.
			static Protocol::Document WritePortMap(const PortReport& report);

		private:
			IPlatform& platform_;
		};
	}
}
