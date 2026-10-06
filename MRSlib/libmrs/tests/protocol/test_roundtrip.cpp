// Round-trip tests, spec 07 §5.1.
#include "doctest.h"

#include "TestFiles.h"
#include "mrs/protocol/Parser.h"
#include "mrs/protocol/Writer.h"

using namespace MRS;

namespace {
	Protocol::WriteOptions OptionsFor(const std::filesystem::path& path) {
		Protocol::WriteOptions options;
		options.compact = path.extension() == ".mrsm";  // message logs use the message form
		return options;
	}

	std::string Describe(const Protocol::ParseError& e) {
		return std::string(Protocol::ErrorClassName(e.error_class)) + " at byte " + std::to_string(e.offset) + " (" + e.header +
		       "): " + e.message;
	}
}

TEST_CASE("spec examples round-trip through the canonical form") {
	const auto files = Test::ProtocolFiles(Test::ExamplesDir());
	REQUIRE(files.size() >= 7);
	for (const auto& path : files) {
		CAPTURE(path.filename().string());
		const auto options = OptionsFor(path);

		// 1. Parse.
		const auto first = Protocol::Parse(Test::ReadFile(path));
		if (!first.Ok()) {
			FAIL_CHECK(Describe(*first.error));
			continue;
		}

		// 2. Write in canonical form; 3. compare with canonical/<name>, byte for byte.
		const std::string written = Protocol::Write(first.document, options);
		const auto canonical_path = Test::ExamplesDir() / "canonical" / path.filename();
		REQUIRE_MESSAGE(std::filesystem::exists(canonical_path), "missing canonical file");
		CHECK(written == Test::ReadFile(canonical_path));

		// 4. Parse the output: the object tree is structurally equal.
		const auto second = Protocol::Parse(written);
		if (!second.Ok()) {
			FAIL_CHECK(Describe(*second.error));
			continue;
		}
		CHECK(second.document == first.document);

		// 5. Write again: byte-identical.
		CHECK(Protocol::Write(second.document, options) == written);
	}
}

TEST_CASE("every canonical file has a source example") {
	for (const auto& path : Test::ProtocolFiles(Test::ExamplesDir() / "canonical")) {
		CAPTURE(path.filename().string());
		CHECK(std::filesystem::exists(Test::ExamplesDir() / path.filename()));
	}
}
