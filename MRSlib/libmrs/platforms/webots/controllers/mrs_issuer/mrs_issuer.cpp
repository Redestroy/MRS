// The task issuer (spec 09 §6) as a Webots supervisor: sends the mission header, dispatches a
// timeline over the radio, and writes per-task results when every task has ended.
//
// Usage (controllerArgs): <mission header .mrs file> <timeline .mrsl> [results.csv] [channel]
// The supervisor needs an Emitter "emitter" and a Receiver "receiver" on the robots' channel
// (default 1). When every task has ended, it writes the results and pauses the simulation.
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <webots/Emitter.hpp>
#include <webots/Receiver.hpp>
#include <webots/Supervisor.hpp>

#include "mrs/algorithms/TaskIssuer.h"
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

	// The supervisor's own radio.
	class RadioTransport : public Comm::ITransport {
	public:
		RadioTransport(webots::Emitter* tx, webots::Receiver* rx) : tx_(tx), rx_(rx) {}
		bool Send(const std::string& text, const std::string&) override {
			return tx_->send(text.c_str(), static_cast<int>(text.size()) + 1) == 1;
		}
		std::vector<std::string> Poll() override {
			std::vector<std::string> out;
			while (rx_->getQueueLength() > 0) {
				const char* data = static_cast<const char*>(rx_->getData());
				const int size = rx_->getDataSize();
				std::string text(data, static_cast<std::size_t>(size));
				while (!text.empty() && text.back() == '\0') text.pop_back();
				out.push_back(text);
				rx_->nextPacket();
			}
			return out;
		}

	private:
		webots::Emitter* tx_;
		webots::Receiver* rx_;
	};
}

int main(int argc, char** argv) {
	if (argc < 3) {
		std::fprintf(stderr, "usage: mrs_issuer <mission header> <timeline.mrsl> [results.csv] [channel]\n");
		return 2;
	}
	const std::string results_path = argc > 3 ? argv[3] : "mrs_issuer_results.csv";
	const int channel = argc > 4 ? std::atoi(argv[4]) : 1;

	webots::Supervisor supervisor;
	const int step = static_cast<int>(supervisor.getBasicTimeStep());
	webots::Emitter* tx = supervisor.getEmitter("emitter");
	webots::Receiver* rx = supervisor.getReceiver("receiver");
	if (!tx || !rx) {
		std::fprintf(stderr, "mrs_issuer needs an Emitter \"emitter\" and a Receiver \"receiver\"\n");
		return 1;
	}
	tx->setChannel(channel);
	rx->setChannel(channel);
	rx->enable(step);
	RadioTransport transport(tx, rx);

	try {
		auto header = Protocol::Parse(ReadFile(argv[1]));
		if (!header.Ok()) throw std::runtime_error(std::string("mission header: ") + header.error->message);
		const Protocol::Record* h = nullptr;
		for (const auto& r : header.document.records)
			if (r.code == "H_M") h = &r;
		if (!h) throw std::runtime_error("no H_M record in " + std::string(argv[1]));

		Algorithms::TaskIssuer issuer(transport, *h);
		issuer.LoadTimeline(ReadFile(argv[2]));
		while (supervisor.step(step) != -1) {
			issuer.Tick(supervisor.getTime());
			if (!issuer.Done()) continue;
			std::ofstream out(results_path, std::ios::binary);
			out << "task,dispatch,done,done_by,done_count,failed\n";
			for (const auto& [id, it] : issuer.Tasks())
				out << id << "," << it.dispatch << "," << (it.done ? std::to_string(*it.done) : "") << "," << it.done_by << ","
				    << it.done_count << "," << it.failed.value_or("") << "\n";
			std::printf("all tasks ended; makespan %.2f s; results in %s\n", issuer.Makespan().value_or(-1.0), results_path.c_str());
			supervisor.simulationSetMode(webots::Supervisor::SIMULATION_MODE_PAUSE);
			break;
		}
	} catch (const std::exception& e) {
		std::fprintf(stderr, "mrs_issuer: %s\n", e.what());
		return 1;
	}
	return 0;
}
