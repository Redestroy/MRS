#pragma once
// Writer for the canonical form of spec 01 §3.5.
#include <string>
#include "mrs/protocol/Record.h"

namespace MRS {
	namespace Protocol {
		struct WriteOptions {
			// Messages (spec 06 §2): all records on one line, separated by nothing.
			bool compact = false;
		};

		std::string Write(const Record& top_level, const WriteOptions& options = {});
		std::string Write(const Document& document, const WriteOptions& options = {});

		// Shortest round-trip text of a number; integral values have no decimal point.
		std::string FormatNumber(double value);
	}
}
