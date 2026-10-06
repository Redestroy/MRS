// Conditions, functions, the behaviour library and requirements (WP1).
#include "doctest.h"

#include <cmath>

#include "../protocol/TestFiles.h"
#include "Harness.h"
#include "mrs/protocol/Parser.h"

using namespace MRS;
using Task::Truth;

namespace {
	constexpr double kPi = 3.14159265358979323846;

	struct Env {
		Task::FunctionRegistry registry;
		Task::TaskFactory factory{registry};
		Environment::Worldview w;
		Env() { Task::RegisterUavFunctions(registry); }

		std::unique_ptr<Task::Condition> Cond(const std::string& text) {
			auto parsed = Protocol::Parse(text);
			REQUIRE(parsed.Ok());
			return factory.BuildCondition(parsed.document.records.at(0));
		}
		Truth Eval(const std::string& text, double t = 0, double context_start = 0, const Environment::GeoReference* geo = nullptr) {
			Task::EvalContext c{w, t, context_start, geo, {}};
			return Cond(text)->Evaluate(c);
		}
	};
}

TEST_CASE("C_L follows LogicalOperation and Kleene rules") {
	Env e;
	e.w.SetBool("a", true, 0);
	e.w.SetBool("b", false, 0);
	auto L = [&](int op, const std::string& p, const std::string& q) {
		return e.Eval("C: C_L " + std::to_string(op) + " C_1 C_2/ C_1: C_? ?_1/ ?_1: " + p + " T/ C_2: C_? ?_1/ ?_1: " + q + " T/");
	};
	CHECK(L(1, "a", "a") == Truth::True);
	CHECK(L(1, "a", "b") == Truth::False);
	CHECK(L(2, "a", "b") == Truth::True);
	CHECK(L(4, "a", "b") == Truth::True);
	CHECK(L(4, "a", "a") == Truth::False);
	CHECK(L(5, "a", "a") == Truth::False);  // NAND
	CHECK(L(6, "b", "b") == Truth::True);   // NOR
	CHECK(L(7, "a", "a") == Truth::True);   // NXOR
	CHECK(e.Eval("C: C_L 3 C_1/ C_1: C_? ?_1/ ?_1: a T/") == Truth::False);
	// Kleene: FALSE AND UNKNOWN is FALSE, TRUE OR UNKNOWN is TRUE, otherwise UNKNOWN.
	CHECK(L(1, "b", "missing") == Truth::False);
	CHECK(L(1, "a", "missing") == Truth::Unknown);
	CHECK(L(2, "a", "missing") == Truth::True);
	CHECK(L(2, "b", "missing") == Truth::Unknown);
	CHECK(L(4, "a", "missing") == Truth::Unknown);
	CHECK(e.Eval("C: C_L 3 C_1/ C_1: C_? ?_1/ ?_1: missing T/") == Truth::Unknown);
}

TEST_CASE("time conditions: C_T in milliseconds, C_W from the context start") {
	Env e;
	CHECK(e.Eval("C: C_T 1500/", 1.499) == Truth::False);
	CHECK(e.Eval("C: C_T 1500/", 1.5) == Truth::True);
	CHECK(e.Eval("C: C_W 2/", 11.9, 10) == Truth::False);
	CHECK(e.Eval("C: C_W 2/", 12.0, 10) == Truth::True);
}

TEST_CASE("position conditions") {
	Env e;
	e.w.SetVec3("pose.enu", 10, 20, 15, 0);
	e.w.SetScalar("att.yaw", kPi - 0.05, 0);
	CHECK(e.Eval("C: C_P3 10.5 20 15.2 1 0.5 0 -1/") == Truth::True);
	CHECK(e.Eval("C: C_P3 10.5 20 16 1 0.5 0 -1/") == Truth::False);
	// Yaw differences wrap: pi - 0.05 and -pi + 0.05 are 0.1 apart.
	CHECK(e.Eval("C: C_P 10 20 -3.0915926535897931 1 0.11/") == Truth::True);
	CHECK(e.Eval("C: C_P 10 20 0 1 0.1/") == Truth::False);
	CHECK(e.Eval("C: C_P3 10 20 15 1 1 0 -1/", 0.6) == Truth::Unknown);  // pose.enu is stale after 0.5 s
	e.w.SetScalar("alt.agl", 14.8, 0);
	CHECK(e.Eval("C: C_H 15 0.5/") == Truth::True);
	CHECK(e.Eval("C: C_m alt.agl gt 15 0/") == Truth::False);
	CHECK(e.Eval("C: C_m alt.agl eq 15 0.25/") == Truth::True);
}

TEST_CASE("C_G converts through the mission's geo reference") {
	Env e;
	const Environment::GeoReference geo(56.9496, 24.1052, 10);
	double lat, lon, alt;
	geo.ToGeodetic({100, 200, 15}, lat, lon, alt);
	const auto back = geo.ToEnu(lat, lon, alt);
	CHECK(back.x == doctest::Approx(100).epsilon(1e-6));
	CHECK(back.y == doctest::Approx(200).epsilon(1e-6));
	CHECK(back.z == doctest::Approx(15).epsilon(1e-6));
	// 0.001 degrees of latitude is about 111 m north.
	CHECK(geo.ToEnu(56.9506, 24.1052, 10).y == doctest::Approx(111.4).epsilon(0.01));

	e.w.SetVec3("pose.enu", 100, 200, 15, 0);
	char text[160];
	std::snprintf(text, sizeof text, "C: C_G %.10f %.10f %.4f 1 0.5/", lat, lon, alt);
	CHECK(e.Eval(text, 0, 0, &geo) == Truth::True);
	CHECK(e.Eval(text, 0, 0, nullptr) == Truth::Unknown);
}

TEST_CASE("C_V matches peers and detections; C_S asks the pool") {
	Env e;
	CHECK(e.Eval("C: C_V V_1/ V_1: V_PEER 0 3 0 0 0 0 0 0 1 0/") == Truth::False);
	e.w.SetVec3("peer.r3.pose.enu", 1, 2, 3, 0);
	CHECK(e.Eval("C: C_V V_1/ V_1: V_PEER 0 3 0 0 0 0 0 0 1 0/") == Truth::True);
	e.w.SetVec3("det.person.1.enu", 50, 50, 0, 0);
	CHECK(e.Eval("C: C_V V_1/ V_1: V_DET 0 person 51 50.5 0 0.9/") == Truth::True);
	CHECK(e.Eval("C: C_V V_1/ V_1: V_DET 0 person 60 50 0 0.9/") == Truth::False);

	auto c = e.Cond("C: C_S op.4 DONE/");
	Task::EvalContext ctx{e.w, 0, 0, nullptr, {}};
	CHECK(c->Evaluate(ctx) == Truth::Unknown);
	ctx.pool_state = [](const std::string& id) -> std::optional<std::string> { return id == "op.4" ? "DONE" : "AVAILABLE"; };
	CHECK(c->Evaluate(ctx) == Truth::True);
}

TEST_CASE("binding writes the target fields of spec 03 §7") {
	Env e;
	e.w.SetVec3("pose.enu", 0, 0, 7, 0);
	e.w.SetScalar("alt.agl", 5, 0);  // ground at 2 m
	auto written = e.Cond("C: C_H 15 0.5/")->Bind(e.w, 0, nullptr);
	CHECK(written == std::vector<std::string>{"target.z", "target.tol_z"});
	CHECK(e.w.Scalar("target.z", 0) == 17.0);
	e.Cond("C: C_P 3 4 1 2 -1/")->Bind(e.w, 0, nullptr);
	CHECK(e.w.Scalar("target.z", 0) == 7.0);  // C_P keeps the current altitude
	e.Cond("C: C_? ?_1/ ?_1: landed T/")->Bind(e.w, 0, nullptr);
	CHECK(e.w.Id("target.predicate", 0) == "landed");
}

TEST_CASE("UAV registry functions (spec 02 §8)") {
	Env e;
	e.w.SetVec3("pose.enu", 0, 0, 5, 0);
	e.w.SetScalar("att.yaw", 0.3, 0);
	e.w.SetScalar("layer.alt", 20, 0);
	e.w.SetScalar("target.x", 30, 0);
	e.w.SetScalar("target.y", 40, 0);
	e.w.SetScalar("target.z", 10, 0);
	auto call = [&](const char* key, std::vector<double> k) { return e.registry.Call(key, e.w, 0, k).value(); };
	CHECK(call("dist_to_target_xy", {}) == doctest::Approx(50));
	CHECK(call("dist_to_target", {}) == doctest::Approx(std::sqrt(2525.0)));
	CHECK(call("yaw_to_target", {}) == doctest::Approx(std::atan2(40, 30)));
	CHECK(call("vel_to_target_x", {0.8, 5}) == doctest::Approx(5));
	// Below the layer and far away: hold x and y, climb to the layer.
	CHECK(call("layered_x", {1, 3}) == 0.0);
	CHECK(call("layered_z", {3}) == 20.0);
	// At the layer: cross.
	e.w.SetVec3("pose.enu", 0, 0, 19.5, 0);
	CHECK(call("layered_x", {1, 3}) == 30.0);
	// Near the target: descend to it.
	e.w.SetVec3("pose.enu", 29, 39, 19.5, 0);
	CHECK(call("layered_z", {3}) == 10.0);
	// Close to the target, yaw keeps the current value.
	e.w.SetVec3("pose.enu", 30, 40, 10, 0);
	CHECK(call("yaw_to_target", {}) == doctest::Approx(0.3));
	// A field it reads is missing: nullopt.
	e.w.Erase("layer.alt");
	CHECK_FALSE(e.registry.Call("layered_z", e.w, 0, {3}).has_value());
}

TEST_CASE("F_X is checked against the registry at load time") {
	Env e;
	auto load = [&](const std::string& text) {
		auto parsed = Protocol::Parse(text);
		REQUIRE(parsed.Ok());
		return e.factory.BuildTask(parsed.document.records.at(0));
	};
	CHECK_THROWS_AS(load("T: T_P op.1 1 0 C_1 C_2 A_1/ C_1: C_N/ C_2: C_N/ A_1: A_FN A_W F_1/ F_1: F_X no_such 0/"), Task::TaskLoadError);
	CHECK_THROWS_AS(load("T: T_P op.1 1 0 C_1 C_2 A_1/ C_1: C_N/ C_2: C_N/ A_1: A_FN A_W F_1/ F_1: F_X layered_z 2 1 2/"), Task::TaskLoadError);
	CHECK_NOTHROW(load("T: T_P op.1 1 0 C_1 C_2 A_1/ C_1: C_N/ C_2: C_N/ A_1: A_FN A_W F_1/ F_1: F_X layered_z 1 3/"));
}

TEST_CASE("the behaviour library picks by code, qualifier, priority and profile") {
	Env e;
	Task::BehaviourLibrary lib;
	lib.PopulateFromFile((Test::ExamplesDir() / "uav_behaviours.mrsb").string(), e.factory);
	CHECK(lib.Entries().size() >= 8);
	CHECK(lib.Find(*e.Cond("C: C_P3 0 0 0 1 1 0 -1/"))->name == "flyto.layered");
	CHECK(lib.Find(*e.Cond("C: C_? ?_1/ ?_1: landed T/"))->name == "land");
	CHECK(lib.Find(*e.Cond("C: C_? ?_1/ ?_1: landed F/")) == nullptr);
	CHECK(lib.Find(*e.Cond("C: C_P 0 0 0 1 -1/")) == nullptr);

	// A robot that cannot read layer.alt... is still fine: layer.alt is a mission field.
	// One that lacks A_PZY gets no fly-to at all.
	Task::CapabilityProfile profile;
	profile.fields = {"pose.enu", "att", "airborne", "alt.agl"};
	profile.actions = {"A_PXY", "A_PZY", "A_TO", "A_LD", "A_HD"};
	CHECK(lib.Find(*e.Cond("C: C_P3 0 0 0 1 1 0 -1/"), &profile)->name == "flyto.layered");
	profile.actions.erase("A_PZY");
	CHECK(lib.Find(*e.Cond("C: C_P3 0 0 0 1 1 0 -1/"), &profile) == nullptr);
}

TEST_CASE("static requirements combine explicit and implicit ones (spec 03 §5)") {
	Env e;
	auto tasks = e.factory.BuildTasks(Test::ReadFile(Test::ExamplesDir() / "uav_point_task.mrst"));
	const auto& task = *tasks.at(0);
	std::vector<std::string> actions;
	task.RequiredActions(actions);
	CHECK(actions == std::vector<std::string>{"A_L", "A_L"});
	Task::CapabilityProfile profile;
	profile.fields = {"pose.enu", "battery"};
	CHECK_FALSE(task.MeetsStaticRequirements(profile));  // no LEDs
	profile.actions = {"A_L"};
	CHECK(task.MeetsStaticRequirements(profile));
	CHECK(task.RuntimeConditions().size() == 1);
	CHECK(task.ToRecord().fields[2].i == 0);
}
