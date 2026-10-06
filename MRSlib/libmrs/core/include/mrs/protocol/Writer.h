#pragma once
// Writer for the canonical form of spec 01 §3.5.
#include <string>
#include "mrs/protocol/Record.h"

namespace MRS {
	namespace Protocol {
		struct WriteOptions {
			// Message form (spec 06 §2): a top-level record and its sub-records on one
			// line, separated by nothing. A document in this form (a .mrsm message log)
			// has one top-level record per line and no blank lines.
			bool compact = false;
		};

		std::string Write(const Record& top_level, const WriteOptions& options = {});
		std::string Write(const Document& document, const WriteOptions& options = {});

		// Shortest round-trip text of a number; integral values have no decimal point.
		std::string FormatNumber(double value);
	}
}
