#pragma once
// Discovery (spec 16 §7.2): a proposed definition and port map from what the providers report.
#include "mrs/device/DeviceRegistry.h"
#include "mrs/device/Identity.h"
#include "mrs/device/Library.h"
#include "mrs/port/Port.h"
#include "mrs/protocol/Record.h"

namespace MRS {
	namespace Device {
		struct DiscoveryOptions {
			const IdentityMap* robot_map = nullptr;  // the robot's .mrsi
			const DeviceLibrary* library = nullptr;  // parts and library identity maps
		};

		struct DiscoveryReport {
			Protocol::Document mrsd;     // the proposed definition
			Protocol::Document mrsp;     // its port map, with the scanned identities
			Protocol::Document unknown;  // I_U records: scanned ports nothing resolves
		};

		// Scans the platform and the provider joints found on it, resolves each port's identity and proposes
		// one node per resolved port. `head` is the D_H record to put on top (its children are ignored).
		// The result is a proposal for the user to check; nothing is built. Provider joints found are
		// opened and started for their own scan, and released before returning.
		DiscoveryReport Discover(const Protocol::Record& head, const DeviceRegistry& registry, Port::IPlatform& platform,
		                         const DiscoveryOptions& options = {});
	}
}
