#pragma once
// A view: one piece of raw information from a sensor, or a decoded peer message (spec 05 §6).
#include <optional>
#include <string>
#include <vector>

#include "mrs/protocol/Record.h"

namespace MRS {
	namespace Environment {
		struct View {
			std::string code;            // "V_GEO"
			double stamp = 0.0;          // mission time of the information
			std::vector<double> values;  // the numeric slots after the stamp, in slot order (V_PEER: robot first)
			std::string text;            // V_DET class, V_PEER task id, V_FLD path
			std::string source;          // not in the record: "peer:rN" for a view a peer shared (spec 15 §2.4)
		};

		// The V record of a view, and back (spec 05 §6). FromRecord returns nullopt for a record that is not a view.
		Protocol::Record ToRecord(const View& view);
		std::optional<View> ViewFromRecord(const Protocol::Record& rec);
	}
}
