// Action packing, spec 07 §5.3 and spec 02 §2.
#include "doctest.h"

#include <cstdint>
#include <vector>

#include "mrs/device/Action.h"
#include "mrs/protocol/Parser.h"

using namespace MRS::Device;

namespace {
	std::uint64_t PackReals(const char* code, std::vector<double> values) {
		auto info = FindAction(code);
		REQUIRE(info.has_value());
		auto arg = PackArgument(info->layout, values);
		REQUIRE(arg.has_value());
		CHECK(UnpackReals(info->layout, *arg) == (info->layout == ArgLayout::NONE ? std::vector<double>{} : values));
		return *arg;
	}

	std::uint64_t PackInts(const char* code, std::vector<std::int64_t> values) {
		auto info = FindAction(code);
		REQUIRE(info.has_value());
		auto arg = PackArgument(info->layout, values);
		REQUIRE(arg.has_value());
		CHECK(UnpackIntegers(info->layout, *arg) == values);
		return *arg;
	}
}

TEST_CASE("fixed packing vectors") {
	CHECK(PackReals("A_PXY", {1, -2}) == 0x3F800000C0000000ull);
	CHECK(PackReals("A_PZY", {15, 1.5}) == 0x417000003FC00000ull);
	CHECK(PackReals("A_W", {2.5}) == 0x4004000000000000ull);
	CHECK(PackInts("A_L", {3, 16711680}) == 0x0000000300FF0000ull);
	CHECK(PackReals("A_MOT", {-409}) == 0xC079900000000000ull);
	CHECK(PackReals("A_N", {}) == 0ull);
}

TEST_CASE("value 1 is the high half") {
	Action a{"A_PXY", PackReals("A_PXY", {120, -40})};
	CHECK(a.FirstHalfAsDouble() == 120.0);
	CHECK(a.SecondHalfAsDouble() == -40.0);
}

TEST_CASE("binary32 rounds to nearest") {
	auto arg = PackArgument(ArgLayout::F32X2, std::vector<double>{0.1, 0});
	REQUIRE(arg.has_value());
	CHECK(UnpackReals(ArgLayout::F32X2, *arg)[0] == static_cast<double>(0.1f));
}

TEST_CASE("values outside the layout's range fail") {
	CHECK_FALSE(PackArgument(ArgLayout::F32X2, std::vector<double>{1e39, 0}).has_value());
	CHECK_FALSE(PackArgument(ArgLayout::U32X2, std::vector<std::int64_t>{-1, 0}).has_value());
	CHECK_FALSE(PackArgument(ArgLayout::U32X2, std::vector<std::int64_t>{4294967296, 0}).has_value());
	CHECK_FALSE(PackArgument(ArgLayout::I32X2, std::vector<std::int64_t>{2147483648, 0}).has_value());
	CHECK_FALSE(PackArgument(ArgLayout::F32X2, std::vector<double>{1}).has_value());     // wrong count
	CHECK_FALSE(PackArgument(ArgLayout::U32X2, std::vector<double>{1, 2}).has_value());  // wrong value type
	const auto parsed = MRS::Protocol::Parse("A: A_PXY 1e39 0/");
	REQUIRE_FALSE(parsed.Ok());
	CHECK(parsed.error->error_class == MRS::Protocol::ErrorClass::Slot);
}

TEST_CASE("I32X2 keeps the sign") {
	auto arg = PackArgument(ArgLayout::I32X2, std::vector<std::int64_t>{-1, 2});
	REQUIRE(arg.has_value());
	CHECK(*arg == 0xFFFFFFFF00000002ull);
	CHECK(UnpackIntegers(ArgLayout::I32X2, *arg) == std::vector<std::int64_t>{-1, 2});
}
