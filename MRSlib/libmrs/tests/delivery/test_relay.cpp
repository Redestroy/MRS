// Package delivery, extension B (spec 18 §6): a range-limited radio in the test simulator and
// message relaying, so robots beyond the issuer's range are reached through a chain.
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>
#include "doctest.h"
#include "../protocol/TestFiles.h"
#include "mrs/comm/Messenger.h"
#include "mrs/sim/Team.h"

using namespace MRS;

namespace {
	// A radio net where only listed pairs hear each other.
	struct Net {
		struct Port : Comm::ITransport {
			Net* net = nullptr;
			std::string name;
			std::vector<std::string> inbox;
			long sent = 0;
			bool Send(const std::string& text, const std::string&) override {
				++sent;
				for (auto& [other, port] : net->ports)
					if (net->links.count({name, other}) || net->links.count({other, name})) port->inbox.push_back(text);
				return true;
			}
			std::vector<std::string> Poll() override {
				auto out = inbox;
				inbox.clear();
				return out;
			}
		};
		std::map<std::string, std::unique_ptr<Port>> ports;
		std::set<std::pair<std::string, std::string>> links;
		Port& Add(const std::string& name) {
			auto p = std::make_unique<Port>();
			p->net = this;
			p->name = name;
			return *(ports[name] = std::move(p));
		}
	};

	Protocol::Record Release(Comm::Messenger& m, const std::string& to, double t) {
		auto r = m.Begin("M_RELEASE", to, t);
		r.fields.push_back(Protocol::Field::MakeText(Protocol::FieldType::TaskId, "op.1"));
		return r;
	}

	Sim::RobotFiles Files() {
		return {Test::ReadFile(Test::ExamplesDir() / "mavic_webots.mrsd"), Test::ReadFile(Test::ExamplesDir() / "mavic_webots.mrsp"),
		        (Test::ExamplesDir() / "uav_behaviours.mrsb").string()};
	}

	// Three robots 25 m apart on a line, the operator 20 m before the first, radio range 30 m:
	// only r1 hears the operator, and each robot hears only its neighbours.
	Sim::TeamConfig ChainConfig(bool relay) {
		Sim::TeamConfig c;
		c.mission.homes = {{0, -5, 0}, {25, -5, 0}, {50, -5, 0}};
		c.radio_range = 30.0;
		c.operator_pos = {-20, -5, 0};
		c.mrs.relay = relay;
		return c;
	}
}

TEST_CASE("relay: a message crosses a chain once per agent, and a unicast reaches its recipient") {
	Net net;
	auto& pa = net.Add("r1");
	auto& pb = net.Add("r2");
	auto& pc = net.Add("r3");
	net.links = {{"r1", "r2"}, {"r2", "r3"}};
	Comm::Messenger a(pa, "r1"), b(pb, "r2"), c(pc, "r3");
	for (auto* m : {&a, &b, &c}) m->SetMission("m1");

	SUBCASE("without relay, r3 never hears r1") {
		a.Post(Release(a, "all", 1.0));
		CHECK(b.Receive().size() == 1);
		CHECK(c.Receive().empty());
	}
	SUBCASE("with relay on r2, r3 hears r1's broadcast once") {
		b.SetRelay(true);
		auto r = Release(a, "all", 1.0);
		a.Post(r);
		CHECK(b.Receive().size() == 1);  // r2 takes it and sends it on
		CHECK(b.Stats().relayed == 1);
		auto got = c.Receive();
		REQUIRE(got.size() == 1);
		CHECK(got[0].sender == "r1");
		CHECK(a.Receive().empty());  // the copy back to r1 is its own
		CHECK(a.Stats().own == 1);
		// Hearing the same message again sends nothing more.
		pb.inbox.push_back(Comm::Encode(r));
		CHECK(b.Receive().empty());
		CHECK(b.Stats().relayed == 1);
	}
	SUBCASE("a unicast from r3 to r1 goes through r2, which does not take it") {
		b.SetRelay(true);
		c.Post(Release(c, "r1", 2.0));
		CHECK(b.Receive().empty());
		CHECK(b.Stats().not_for_us == 1);
		CHECK(b.Stats().relayed == 1);
		CHECK(a.Receive().size() == 1);
	}
	SUBCASE("a unicast to the relay itself is not sent on") {
		b.SetRelay(true);
		a.Post(Release(a, "r2", 2.0));
		CHECK(b.Receive().size() == 1);
		CHECK(b.Stats().relayed == 0);
	}
}

TEST_CASE("range-limited radio: robots beyond the issuer's range learn the mission only through relays") {
	SUBCASE("without relay") {
		Sim::Team team(Files(), ChainConfig(false));
		team.Run(5.0, false);
		CHECK_FALSE(team.robots[0]->layer->Messages().Mission().empty());
		CHECK(team.robots[1]->layer->Messages().Mission().empty());
		CHECK(team.robots[2]->layer->Messages().Mission().empty());
		CHECK(team.air.out_of_range > 0);
	}
	SUBCASE("with relay") {
		Sim::Team team(Files(), ChainConfig(true));
		team.Run(5.0, false);
		for (auto& r : team.robots) CHECK_FALSE(r->layer->Messages().Mission().empty());
		// r3's heartbeats reach the issuer through r2 and r1.
		CHECK(team.robots[1]->layer->Messages().Stats().relayed > 0);
		CHECK(team.robots[0]->layer->Messages().Stats().relayed > 0);
	}
}
