// Unit tests for parser and writer rules of spec 01 that the example files do not cover.
#include "doctest.h"

#include "mrs/protocol/Parser.h"
#include "mrs/protocol/Writer.h"

using namespace MRS;

namespace {
	std::string Canonical(const std::string& text, bool compact = false) {
		const auto result = Protocol::Parse(text);
		if (!result.Ok()) return std::string("error: ") + result.error->message;
		Protocol::WriteOptions options;
		options.compact = compact;
		return Protocol::Write(result.document, options);
	}

	Protocol::ErrorClass ErrorOf(const std::string& text) {
		const auto result = Protocol::Parse(text);
		REQUIRE_FALSE(result.Ok());
		return result.error->error_class;
	}
}

TEST_CASE("labels are renumbered per kind in reference order") {
	CHECK(Canonical("T:T_A op.1 1.0 0 C_3 C_9 A_4 A_5 A_6/ C_3:C_N/ C_9: C_L 3 C_1/ C_1: C_? ?_7/ ?_7: airborne F/\n"
	                "A_4: A_W 2.50/ A_5: A_PXY 1 -2/ A_6: A_N/") ==
	      "T: T_A op.1 1 0 C_1 C_2 A_1..3/\n"
	      "C_1: C_N/\n"
	      "C_2: C_L 3 C_1/\n"
	      "C_1: C_? ?_1/\n"
	      "?_1: airborne F/\n"
	      "A_1: A_W 2.5/\n"
	      "A_2: A_PXY 1 -2/\n"
	      "A_3: A_N/\n");
}

TEST_CASE("a range may fill several slots") {
	const std::string behaviour = "T: T_B 0 1 0 C_1..3 T_1/\nC_1: C_N/\nC_2: C_N/\nC_3: C_W 5/\n"
	                              "T_1: T_A 0 1 0 C_1 C_2 A_1/\nC_1: C_N/\nC_2: C_N/\nA_1: A_HD 0 0/\n";
	CHECK(Canonical(behaviour) == behaviour);
	CHECK(Canonical("T: T_B 0 1 0 C_1 C_2 C_3 T_1/\nC_1: C_N/\nC_2: C_N/\nC_3: C_W 5/\n"
	                "T_1: T_A 0 1 0 C_1 C_2 A_1/\nC_1: C_N/\nC_2: C_N/\nA_1: A_HD 0 0/\n") == behaviour);
}

TEST_CASE("two labels stay a list, three become a range") {
	CHECK(Canonical("T: T_A op.1 1 0 C_1 C_2 A_1..2/ C_1: C_N/ C_2: C_N/ A_1: A_W 1/ A_2: A_N/") ==
	      "T: T_A op.1 1 0 C_1 C_2 A_1 A_2/\nC_1: C_N/\nC_2: C_N/\nA_1: A_W 1/\nA_2: A_N/\n");
}

TEST_CASE("labels are local to their parent") {
	CHECK(Protocol::Parse("C: C_L 1 C_1 C_2/\nC_1: C_L 2 C_1 C_2/\nC_1: C_N/\nC_2: C_F/\nC_2: C_N/").Ok());
}

TEST_CASE("numbers use the shortest round-trip form") {
	CHECK(Protocol::FormatNumber(1.0) == "1");
	CHECK(Protocol::FormatNumber(0.1) == "0.1");
	CHECK(Protocol::FormatNumber(-0.5) == "-0.5");
	CHECK(Protocol::FormatNumber(1e21) == "1e+21");
	CHECK(Canonical("C: C_H 15.000 .5/") == "C: C_H 15 0.5/\n");
	CHECK(Canonical("C: C_H -0 1e-3/") == "C: C_H -0 0.001/\n");
	CHECK(Canonical("C: C_W 1./") == "C: C_W 1/\n");
}

TEST_CASE("malformed numbers are lexical errors") {
	CHECK(ErrorOf("C: C_W +1/") == Protocol::ErrorClass::Lexical);
	CHECK(ErrorOf("C: C_W 0x10/") == Protocol::ErrorClass::Lexical);
	CHECK(ErrorOf("C: C_W 1e400/") == Protocol::ErrorClass::Lexical);
	CHECK(ErrorOf("C: C_W INF/") == Protocol::ErrorClass::Lexical);
	CHECK(ErrorOf("C: C_T 99999999999999999999/") == Protocol::ErrorClass::Lexical);
}

TEST_CASE("strings keep spaces, slashes and escapes") {
	const std::string text = "P: P_R SIM \"front left/right \\\"led\\\" \\\\\" T 1 device \"a b\"/\n";
	const auto result = Protocol::Parse(text);
	REQUIRE(result.Ok());
	CHECK(result.document.records[0].fields[1].s == "front left/right \"led\" \\");
	CHECK(Protocol::Write(result.document) == text);
	CHECK(ErrorOf("P: P_R SIM \"bad \\n\" T 0/") == Protocol::ErrorClass::Lexical);
	CHECK(ErrorOf("P: P_R SIM \"a\"b T 0/") == Protocol::ErrorClass::Lexical);
}

TEST_CASE("comments are allowed between records only") {
	CHECK(Protocol::Parse("# a comment\nC: C_N/   # after a record\n# another\n").Ok());
	CHECK(ErrorOf("C: C_L 1 # inside\n C_1 C_2/\nC_1: C_N/\nC_2: C_N/") == Protocol::ErrorClass::Lexical);
}

TEST_CASE("lower-case names are identifiers, not codes") {
	CHECK(Protocol::Parse("A: A_MAP m_fl A_1 m_fr A_2/\nA_1: A_MOT 412.5/\nA_2: A_MOT -409/").Ok());
}

TEST_CASE("slot rules") {
	CHECK(ErrorOf("T: T_A op.1 0 0 C_1 C_2 A_1/ C_1: C_N/ C_2: C_N/ A_1: A_N/") == Protocol::ErrorClass::Slot);  // priority > 0
	CHECK(ErrorOf("T: T_A op.1 1 7 C_1 C_2 A_1/ C_1: C_N/ C_2: C_N/ A_1: A_N/") == Protocol::ErrorClass::Slot);  // state 0-6
	CHECK(ErrorOf("C: C_L 3 C_1 C_2/ C_1: C_N/ C_2: C_N/") == Protocol::ErrorClass::Slot);  // NOT takes one child
	CHECK(ErrorOf("C: C_L 1 C_1/ C_1: C_N/") == Protocol::ErrorClass::Slot);              // AND takes two or more
	CHECK(ErrorOf("C: C_T 1.5/") == Protocol::ErrorClass::Slot);                           // milliseconds, an integer
	CHECK(ErrorOf("C: C_m pose.enu.z about 1 0/") == Protocol::ErrorClass::Slot);
	CHECK(ErrorOf("C: C_S op.1 FINISHED/") == Protocol::ErrorClass::Slot);
	CHECK(ErrorOf("A: A_L -1 0/") == Protocol::ErrorClass::Slot);                          // uint32 range
	CHECK(ErrorOf("A: A_MAP any A_1 any A_2/ A_1: A_PXY 1 2/ A_2: A_PXY 3 4/") == Protocol::ErrorClass::Slot);
	CHECK(ErrorOf("M: M_STATE 0.1 m1 r1 all 1 0 V_1/ V_1: V_GEO 0 1 2 3/") == Protocol::ErrorClass::Slot);
	CHECK(ErrorOf("C: C_P3 1 2 3 4 5 6 7 8/") == Protocol::ErrorClass::Slot);
}

TEST_CASE("code rules") {
	CHECK(ErrorOf("X: X_A/") == Protocol::ErrorClass::Code);        // reserved kind
	CHECK(ErrorOf("T: C_N/") == Protocol::ErrorClass::Code);        // code of another kind
	CHECK(ErrorOf("F: F_E 1/") == Protocol::ErrorClass::Code);      // not planned
	CHECK(ErrorOf("C: C_Q/") == Protocol::ErrorClass::Code);        // reserved
	CHECK(ErrorOf("R: R_A A_D/") == Protocol::ErrorClass::Code);    // legacy action in a code slot
}

TEST_CASE("reference rules") {
	CHECK(ErrorOf("C_1: C_N/") == Protocol::ErrorClass::Reference);                  // sub-record without a parent
	CHECK(ErrorOf("C: C_L 1 C_1 C_2/ C_1: C_N/") == Protocol::ErrorClass::Reference);  // missing at the end
	CHECK(ErrorOf("C: C_L 1 C_1 C_2/ C_1: C_N/ C_3: C_N/") == Protocol::ErrorClass::Reference);
}

TEST_CASE("version record") {
	CHECK(Protocol::Parse("@: MRS 0.1/").Ok());
	CHECK(Protocol::Parse("@: MRS 0.7/").Ok());
	CHECK(ErrorOf("@: MRS 2/") == Protocol::ErrorClass::Version);
	CHECK(ErrorOf("@: ROS 0.1/") == Protocol::ErrorClass::Slot);
}

TEST_CASE("message form puts each message on one line") {
	const std::string text = "M: M_STATE 0.1 m1 r2 all 58 142.5 V_1/\nV_1: V_PEER 142.48 2 1 2 3 0 0 0 0.71 op.17/\n"
	                         "M: M_DONE 0.1 m1 r2 all 59 143 op.17/\n";
	CHECK(Canonical(text, true) ==
	      "M: M_STATE 0.1 m1 r2 all 58 142.5 V_1/V_1: V_PEER 142.48 2 1 2 3 0 0 0 0.71 op.17/\n"
	      "M: M_DONE 0.1 m1 r2 all 59 143 op.17/\n");
	const auto result = Protocol::Parse(text);
	REQUIRE(result.Ok());
	Protocol::WriteOptions compact;
	compact.compact = true;
	CHECK(Protocol::Write(result.document.records[1], compact) == "M: M_DONE 0.1 m1 r2 all 59 143 op.17/");
}

TEST_CASE("structural equality ignores label names and offsets, not values") {
	const auto a = Protocol::Parse("C: C_L 1 C_1 C_2/ C_1: C_N/ C_2: C_H 1 0/");
	const auto b = Protocol::Parse("\n\nC:C_L 1 C_5 C_8/\nC_5: C_N/\nC_8: C_H 1.0 0.0/");
	const auto c = Protocol::Parse("C: C_L 1 C_1 C_2/ C_1: C_N/ C_2: C_H 1 -0/");
	REQUIRE((a.Ok() && b.Ok() && c.Ok()));
	CHECK(a.document == b.document);
	CHECK(a.document != c.document);
}
