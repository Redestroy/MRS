// mrs_fmt: checks a protocol file and prints its canonical form (spec 01 §3.5).
// Usage: mrs_fmt <file>     Exit code 0 when the file is valid, 1 otherwise.
#include <fstream>
#include <iostream>
#include <sstream>

#include "mrs/protocol/Parser.h"
#include "mrs/protocol/Writer.h"

int main(int argc, char** argv) {
	if (argc != 2) {
		std::cerr << "usage: mrs_fmt <file>\n";
		return 2;
	}
	std::ifstream in(argv[1], std::ios::binary);
	if (!in) {
		std::cerr << "cannot open " << argv[1] << "\n";
		return 2;
	}
	std::stringstream buffer;
	buffer << in.rdbuf();
	const auto result = MRS::Protocol::Parse(buffer.str());
	if (!result.Ok()) {
		const auto& e = *result.error;
		std::cerr << argv[1] << ": " << MRS::Protocol::ErrorClassName(e.error_class) << " at byte " << e.offset;
		if (!e.header.empty()) std::cerr << " (record " << e.header << ")";
		std::cerr << ": " << e.message << "\n";
		return 1;
	}
	std::cout << MRS::Protocol::Write(result.document);
	return 0;
}
