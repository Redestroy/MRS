// The task issuer (spec 09 §6) as a Webots supervisor: sends the mission header, dispatches a
// timeline over the radio, and writes per-task results when every task has ended.
//
// Usage (controllerArgs): <mission header .mrs file> <timeline .mrsl> [results.csv] [channel]
//                         [--track <track.csv>|none] [--track-period <s>]
// The supervisor needs an Emitter "emitter" and a Receiver "receiver" on the robots' channel
// (default 1). When every task has ended, it writes the results and pauses the simulation.
// It also logs the pose of every other robot in the world to a track file for the MRS dot viewer
// (spec 17 §2), by default <results>_track.csv every 0.1 s, with the task events.
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <webots/Emitter.hpp>
#include <webots/Receiver.hpp>
#include <webots/Supervisor.hpp>

#include "mrs/algorithms/TaskIssuer.h"
#include "mrs/protocol/Parser.h"
#include "mrs/replay/Track.h"

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
			return tx_->send(text.data(), static_cast<int>(text.size())) == 1;
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

	// The robots the track follows: every Robot node at the top of the scene tree except this one.
	struct Tracked {
		std::vector<webots::Node*> nodes;
		std::vector<std::string> names;
	};

	Tracked FindRobots(webots::Supervisor& supervisor) {
		Tracked t;
		webots::Field* children = supervisor.getRoot()->getField("children");
		webots::Node* self = supervisor.getSelf();
		for (int k = 0; k < children->getCount(); ++k) {
			webots::Node* n = children->getMFNode(k);
			if (!n || n->getBaseTypeName() != "Robot" || n->getId() == self->getId()) continue;
			webots::Field* name = n->getField("name");
			t.nodes.push_back(n);
			t.names.push_back(name ? name->getSFString() : "robot" + std::to_string(k));
		}
		return t;
	}

	std::vector<Replay::Pose> Poses(const Tracked& t) {
		std::vector<Replay::Pose> out;
		for (webots::Node* n : t.nodes) {
			const double* p = n->getPosition();
			Replay::Pose pose{p[0], p[1], p[2], 0, 0, 0};
			Replay::RotationToRpy(n->getOrientation(), pose.roll, pose.pitch, pose.yaw);
			out.push_back(pose);
		}
		return out;
	}
}

int main(int argc, char** argv) {
	std::vector<std::string> args;
	std::string track_path;
	double track_period = 0.1;
	for (int k = 1; k < argc; ++k) {
		const std::string a = argv[k];
		if ((a == "--track" || a == "--track-period") && k + 1 < argc) {
			if (a == "--track") track_path = argv[++k];
			else track_period = std::atof(argv[++k]);
		} else {
			args.push_back(a);
		}
	}
	if (args.size() < 2) {
		std::fprintf(stderr, "usage: mrs_issuer <mission header> <timeline.mrsl> [results.csv] [channel] [--track <file>|none] [--track-period <s>]\n");
		return 2;
	}
	std::setvbuf(stdout, nullptr, _IONBF, 0);  // Webots shows a controller's output only as it is flushed
	const std::string results_path = args.size() > 2 ? args[2] : "mrs_issuer_results.csv";
	const int channel = args.size() > 3 ? std::atoi(args[3].c_str()) : 1;
	if (track_path.empty()) {
		const auto dot = results_path.rfind('.');
		const auto slash = results_path.find_last_of("/\\");
		const bool ext = dot != std::string::npos && (slash == std::string::npos || dot > slash);
		track_path = (ext ? results_path.substr(0, dot) : results_path) + "_track.csv";
	}

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
		auto header = Protocol::Parse(ReadFile(args[0]));
		if (!header.Ok()) throw std::runtime_error(std::string("mission header: ") + header.error->message);
		const Protocol::Record* h = nullptr;
		for (const auto& r : header.document.records)
			if (r.code == "H_M") h = &r;
		if (!h) throw std::runtime_error("no H_M record in " + args[0]);

		Algorithms::TaskIssuer issuer(transport, *h);
		issuer.LoadTimeline(ReadFile(args[1]));

		// The track (spec 17 §2): a row every track_period, flushed every row so a stopped run keeps it.
		std::ofstream track_file;
		std::unique_ptr<Replay::TrackWriter> track;
		Replay::IssuerEvents events;
		const Tracked robots = FindRobots(supervisor);
		if (track_path != "none") {
			track_file.open(track_path, std::ios::binary);
			if (!track_file) throw std::runtime_error("cannot write " + track_path);
			track = std::make_unique<Replay::TrackWriter>(track_file, robots.names, h);
			std::printf("tracking %zu robots in %s\n", robots.names.size(), track_path.c_str());
		}
		double next_row = -1.0;
		auto log = [&](double t) {
			if (!track) return;
			events.Poll(issuer, *track, t);
			if (t + 1e-9 < next_row) return;
			track->Sample(t, Poses(robots));
			track_file.flush();
			next_row = t + track_period;
		};

		while (supervisor.step(step) != -1) {
			const double t = supervisor.getTime();
			issuer.Tick(t);
			log(t);
			if (!issuer.Done()) continue;
			next_row = -1.0;
			log(t);  // the last row, at the end
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
