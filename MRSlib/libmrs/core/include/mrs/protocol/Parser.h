#pragma once
// Parser for the MRS text protocol, version 0.1 (spec 01).
// Checks the lexical rules, the record structure, the depth-first rule and the
// slot list of every code in specs 02-06. It never guesses or repairs input.
#include <cstddef>
#include <optional>
#include <string>
#include <string_view>
#include "mrs/protocol/Record.h"

namespace MRS {
	namespace Protocol {
		// Error classes of spec 01 §5.
		enum class ErrorClass { Lexical, Code, Slot, Reference, Version, Trailing };

		// Stable names used in test files: lexical_error, code_error, slot_error,
		// reference_error, version_error, trailing_data.
		const char* ErrorClassName(ErrorClass c);

		struct ParseError {
			ErrorClass error_class = ErrorClass::Lexical;
			std::size_t offset = 0;   // byte offset where the error was found
			std::string header;       // header of the record being parsed, if any
			std::string message;
		};

		struct ParseResult {
			Document document;
			std::optional<ParseError> error;
			bool Ok() const { return !error.has_value(); }
		};

		ParseResult Parse(std::string_view text);
	}
}
