// Smoke tests for the WP0 skeleton. The full suite of spec 07 §5 (canonical files,
// one invalid file per error class) is written after JB's review of the skeleton.
#include "doctest.h"

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include "mrs/device/Action.h"
#include "mrs/protocol/Parser.h"
#include "mrs/protocol/Writer.h"

using namespace MRS;

namespace {
	std::string ReadFile(const std::filesystem::path& path) {
		std::ifstream in(path, std::ios::binary);
		std::stringstream buffer;
		buffer << in.rdbuf();
		return buffer.str();
	}

	Protocol::ErrorClass ErrorOf(const std::string& text) {
		auto result = Protocol::Parse(text);
		REQUIRE_FALSE(result.Ok());
		return result.error->error_class;
	}
}

TEST_CASE("every spec example parses and round-trips") {
	int files = 0;
	for (const auto& entry : std::filesystem::directory_iterator(MRS_SPEC_EXAMPLES)) {
		if (!entry.is_regular_file() || entry.path().extension().string().rfind(".mrs", 0) != 0) continue;
		CAPTURE(entry.path().filename().string());
		const auto first = Protocol::Parse(ReadFile(entry.path()));
		if (!first.Ok()) FAIL(std::string(Protocol::ErrorClassName(first.error->error_class)), " at ", first.error->offset, ": ", first.error->message);
		const std::string written = Protocol::Write(first.document);
		const auto second = Protocol::Parse(written);
		REQUIRE(second.Ok());
		CHECK(second.document == first.document);
		CHECK(Protocol::Write(second.document) == written);
		++files;
	}
	CHECK(files >= 7);
}

TEST_CASE("canonical form") {
	const auto result = Protocol::Parse("T:T_A op.1 1.0 0 C_3 C_9 A_4 A_5 A_6/ C_3:C_N/ C_9: C_L 3 C_1/ C_1: C_? ?_7/ ?_7: airborne F/\n"
	                                    "A_4: A_W 2.50/ A_5: A_PXY 1 -2/ A_6: A_N/");
	REQUIRE(result.Ok());
	CHECK(Protocol::Write(result.document) ==
	      "T: T_A op.1 1 0 C_1 C_2 A_1..3/\n"
	      "C_1: C_N/\n"
	      "C_2: C_L 3 C_1/\n"
	      "C_1: C_? ?_1/\n"
	      "?_1: airborne F/\n"
	      "A_1: A_W 2.5/\n"
	      "A_2: A_PXY 1 -2/\n"
	      "A_3: A_N/\n");
}

TEST_CASE("error classes") {
	CHECK(ErrorOf("C: C_N $/") == Protocol::ErrorClass::Lexical);
	CHECK(ErrorOf("C: C_W nan/") == Protocol::ErrorClass::Lexical);
	CHECK(ErrorOf("T: T_Z/") == Protocol::ErrorClass::Code);
	CHECK(ErrorOf("T: T_G/") == Protocol::ErrorClass::Code);
	CHECK(ErrorOf("A: A_D 1/") == Protocol::ErrorClass::Code);
	CHECK(ErrorOf("C: C_P3 1 2 3 4 5/") == Protocol::ErrorClass::Slot);
	CHECK(ErrorOf("C: C_L 8 C_1 C_2/\nC_1: C_N/\nC_2: C_N/") == Protocol::ErrorClass::Slot);
	CHECK(ErrorOf("C: C_L 1 C_1 C_2/\nC_1: C_N/") == Protocol::ErrorClass::Reference);
	CHECK(ErrorOf("C: C_L 1 C_1 C_2/\nC_2: C_N/\nC_1: C_N/") == Protocol::ErrorClass::Reference);
	CHECK(ErrorOf("C: C_L 1 C_1 C_1/\nC_1: C_N/\nC_1: C_N/") == Protocol::ErrorClass::Reference);
	CHECK(ErrorOf("@: MRS 1.0/") == Protocol::ErrorClass::Version);
	CHECK(ErrorOf("C: C_N/\nC: C_N") == Protocol::ErrorClass::Trailing);
	CHECK(ErrorOf("A: A_MAP any A_1/\nA_1: A_MAP any A_1/\nA_1: A_N/") == Protocol::ErrorClass::Slot);
	CHECK(ErrorOf("A: A_FN A_PXY F_1/\nF_1: F_K 1/") == Protocol::ErrorClass::Slot);
	CHECK(ErrorOf("A: A_PXY 1e39 0/") == Protocol::ErrorClass::Slot);
}

TEST_CASE("action packing vectors (spec 07 §5.3)") {
	using Device::ArgLayout;
	CHECK(*Device::PackArgument(ArgLayout::F32X2, std::vector<double>{1, -2}) == 0x3F800000C0000000ull);
	CHECK(*Device::PackArgument(ArgLayout::F32X2, std::vector<double>{15, 1.5}) == 0x417000003FC00000ull);
	CHECK(*Device::PackArgument(ArgLayout::F64, std::vector<double>{2.5}) == 0x4004000000000000ull);
	CHECK(*Device::PackArgument(ArgLayout::U32X2, std::vector<std::int64_t>{3, 16711680}) == 0x0000000300FF0000ull);
	CHECK(*Device::PackArgument(ArgLayout::F64, std::vector<double>{-409}) == 0xC079900000000000ull);
	CHECK(*Device::PackArgument(ArgLayout::NONE, std::vector<double>{}) == 0ull);
	CHECK_FALSE(Device::PackArgument(ArgLayout::F32X2, std::vector<double>{1e39, 0}).has_value());
	CHECK(Device::UnpackReals(ArgLayout::F32X2, 0x3F800000C0000000ull) == std::vector<double>{1, -2});
}
