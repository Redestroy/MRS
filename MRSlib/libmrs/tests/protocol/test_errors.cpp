// Error tests, spec 07 §5.2: each file in spec/examples/invalid/ starts with
// "# expect: <error class> <byte offset>" and must fail with exactly that.
#include "doctest.h"

#include <sstream>

#include "TestFiles.h"
#include "mrs/protocol/Parser.h"

using namespace MRS;

TEST_CASE("invalid files fail with the expected class at the expected offset") {
	const auto files = Test::ProtocolFiles(Test::ExamplesDir() / "invalid");
	REQUIRE(files.size() >= 15);
	for (const auto& path : files) {
		CAPTURE(path.filename().string());
		const std::string text = Test::ReadFile(path);
		std::istringstream first_line(text.substr(0, text.find('\n')));
		std::string hash, expect, error_class;
		std::size_t offset = 0;
		first_line >> hash >> expect >> error_class >> offset;
		REQUIRE_MESSAGE((hash == "#" && expect == "expect:" && !error_class.empty()), "first line must be '# expect: <class> <offset>'");

		const auto result = Protocol::Parse(text);
		REQUIRE_FALSE(result.Ok());
		CHECK(std::string(Protocol::ErrorClassName(result.error->error_class)) == error_class);
		CHECK(result.error->offset == offset);
	}
}
