// The task issuer (spec 09 §6) as a program on UDP (spec 14 §8): sends the mission header and a
// timeline to robots on ArduPilot (mrs_ardupilot_uav), and writes per-task results at the end.
//
// Usage: mrs_operator <mission header .mrs> <timeline .mrsl> [options]
//   --results FILE      (default mrs_operator_results.csv)
//   --port P            UDP port the operator listens on (default 14600)
//   --peers H:P,H:P     where messages go (default 127.0.0.1:14601 .. 14600 + --team)
//   --team N            team size for the default peers (default 8)
//   --multicast GROUP   use a multicast group on --port instead of peers
//   --epoch UNIX_S      mission time 0 (default: the last UTC midnight); every agent needs the same
//   --delay S           seconds after start before the timeline's time 0 (default 5)
//   --limit S           give up after this long (default 1800)
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>

#include "mrs/algorithms/TaskIssuer.h"
#include "mrs/platform/UdpTransport.h"
#include "mrs/protocol/Parser.h"

using namespace MRS;

namespace {
	std::string ReadFile(const std::string& path) {
		std::ifstream in(path, std::ios::binary);
		if (!in) throw std::runtime_error("cannot open " + path);
		std::stringstream s;
		s << in.rdbuf();
		return s.str();
	}

	std::vector<std::string> Split(const std::string& text, char sep) {
		std::vector<std::string> out;
		std::stringstream s(text);
		std::string item;
		while (std::getline(s, item, sep))
			if (!item.empty()) out.push_back(item);
		return out;
	}

	double UnixNow() { return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count(); }
}

int main(int argc, char** argv) {
	if (argc < 3) {
		std::fprintf(stderr, "usage: mrs_operator <mission header> <timeline.mrsl> [--results F] [--port P] [--peers H:P,..] [--team N]\n"
		                     "       [--multicast G] [--epoch S] [--delay S] [--limit S]\n");
		return 2;
	}
	std::setvbuf(stdout, nullptr, _IONBF, 0);
	std::string results = "mrs_operator_results.csv", peers, multicast;
	int port = 14600, team = 8;
	double epoch = -1.0, delay = 5.0, limit = 1800.0;
	for (int k = 3; k + 1 < argc; k += 2) {
		const std::string key = argv[k], value = argv[k + 1];
		if (key == "--results") results = value;
		else if (key == "--port") port = std::atoi(value.c_str());
		else if (key == "--peers") peers = value;
		else if (key == "--team") team = std::atoi(value.c_str());
		else if (key == "--multicast") multicast = value;
		else if (key == "--epoch") epoch = std::atof(value.c_str());
		else if (key == "--delay") delay = std::atof(value.c_str());
		else if (key == "--limit") limit = std::atof(value.c_str());
		else {
			std::fprintf(stderr, "unknown option %s\n", key.c_str());
			return 2;
		}
	}
	try {
		auto header = Protocol::Parse(ReadFile(argv[1]));
		if (!header.Ok()) throw std::runtime_error(std::string("mission header: ") + header.error->message);
		const Protocol::Record* h = nullptr;
		for (const auto& r : header.document.records)
			if (r.code == "H_M") h = &r;
		if (!h) throw std::runtime_error("no H_M record in " + std::string(argv[1]));

		Net::UdpTransportConfig uc;
		uc.port = port;
		uc.multicast = multicast;
		if (multicast.empty()) {
			if (peers.empty())
				for (int k = 1; k <= team; ++k) uc.peers.push_back({"127.0.0.1", 14600 + k});
			else
				for (const auto& p : Split(peers, ',')) uc.peers.push_back(Net::ParseEndpoint(p));
		}
		Net::UdpTransport transport(uc);
		if (!transport.Ok()) throw std::runtime_error("cannot open UDP port " + std::to_string(port));

		const double unix0 = UnixNow();
		if (epoch < 0.0) epoch = std::floor(unix0 / 86400.0) * 86400.0;
		auto now = [&] { return UnixNow() - epoch; };
		const double start = now() + delay;

		Algorithms::TaskIssuer issuer(transport, *h);
		// The timeline's times count from `start` (LoadTimeline, shifted).
		auto timeline = Protocol::Parse(ReadFile(argv[2]));
		if (!timeline.Ok()) throw std::runtime_error(std::string("timeline: ") + timeline.error->message);
		for (const auto& r : timeline.document.records) {
			if (r.code != "L_D") continue;
			std::vector<Protocol::Record> tasks;
			for (std::size_t k = 1; k < r.fields.size(); ++k)
				if (r.fields[k].type == Protocol::FieldType::Ref) tasks.push_back(r.children.at(r.fields[k].ref));
			issuer.Schedule(start + r.fields[0].n, tasks);
		}
		std::printf("operator: mission %s, timeline from t = %.2f, listening on %d\n", h->fields[0].s.c_str(), start, port);

		double next_status = 0.0;
		const double give_up = now() + limit;
		while (!issuer.Done()) {
			const double t = now();
			issuer.Tick(t);
			if (t >= next_status) {
				next_status = t + 10.0;
				int done = 0;
				for (const auto& [id, it] : issuer.Tasks())
					if (it.done || it.failed) ++done;
				std::printf("%10.2f operator: %d of %zu tasks ended, %ld messages in\n", t, done, issuer.Tasks().size(),
				            issuer.Messages().Stats().received);
			}
			if (t > give_up) {
				std::fprintf(stderr, "operator: time limit reached\n");
				break;
			}
			transport.Wait(0.05);
		}
		std::ofstream out(results, std::ios::binary);
		out << "task,dispatch,done,done_by,done_count,failed\n";
		for (const auto& [id, it] : issuer.Tasks())
			out << id << "," << it.dispatch - start << "," << (it.done ? std::to_string(*it.done - start) : "") << "," << it.done_by << ","
			    << it.done_count << "," << it.failed.value_or("") << "\n";
		std::printf("operator: %s; makespan %.2f s; results in %s\n", issuer.Done() ? "all tasks ended" : "stopped",
		            issuer.Makespan().value_or(-1.0), results.c_str());
		// Keep answering for a few seconds, so robots hear the last messages.
		const double end = now() + 3.0;
		while (now() < end) {
			issuer.Tick(now());
			transport.Wait(0.05);
		}
		return issuer.Done() ? 0 : 1;
	} catch (const std::exception& e) {
		std::fprintf(stderr, "mrs_operator: %s\n", e.what());
		return 1;
	}
}
