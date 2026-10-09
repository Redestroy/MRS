// mrs_experiment: the WP6 batch runner (spec 10 §6). It flies task sets in the test simulator
// under each condition and seed and appends one CSV row per run.
//
//   mrs_experiment run [options]       run a grid (see Usage)
//   mrs_experiment gen F D N SEED      print a generated task set (family, dispatch, tasks, seed)
//   mrs_experiment oracle FILE         print the oracle form of a timeline
#include <algorithm>
#include <atomic>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "mrs/sim/Experiment.h"
#include "mrs/sim/TaskSets.h"

namespace fs = std::filesystem;
using namespace MRS;

namespace {
	const char* kUsage =
	    "usage:\n"
	    "  mrs_experiment run [--ported DIR] [--gen FAMILY:DISPATCH:TASKS]... [--seeds A-B]\n"
	    "                     [--cond S1|S1*|G-RTA:N,..|G-RTA-X:N,..|G-C:N,..]... [--jobs J]\n"
	    "                     [--out results.csv] [--limit SECONDS] [--sets-out DIR] [--examples DIR]\n"
	    "                     [--commit HASH] [--trees FAMILY:TREES:PARTS[:INTERVAL]]... [--cond G-STA:N,..|G-CBBA:N,..]\n"
	    "                     [--bitrate BPS,..]  (0: no limit; every run is flown at each bitrate)\n"
	    "                     [--repulsion LEAD,..]  (s; 0: off; every run is flown at each lead, spec 15 §4.3)\n"
	    "                     [--ranging NOISE_M]  (every robot gets a ranging sensor, spec 15 §3.1)\n"
	    "  mrs_experiment gen FAMILY DISPATCH TASKS SEED\n"
	    "  mrs_experiment oracle FILE\n"
	    "families: grid radial cluster multicluster random; dispatch: static even clustered random\n"
	    "tree families (spec 12 §4): coverage perimeter search mixed\n";

	std::string ReadFile(const fs::path& p) {
		std::ifstream in(p, std::ios::binary);
		if (!in) throw std::runtime_error("cannot open " + p.string());
		std::stringstream s;
		s << in.rdbuf();
		return s.str();
	}

	struct GenSpec {
		Sim::Family family;
		Sim::Dispatch dispatch;
		int tasks;
	};

	struct TreeSpec {
		Sim::TreeFamily family;
		int trees, parts;
		double interval;  // s between releases; 0: the default
	};

	struct CondSpec {
		Sim::Condition condition;
		std::vector<int> n;
	};

	std::vector<int> Ints(const std::string& s) {
		std::vector<int> out;
		std::stringstream in(s);
		std::string part;
		while (std::getline(in, part, ',')) out.push_back(std::stoi(part));
		return out;
	}

	std::string Fmt(double v) {
		std::ostringstream o;
		o << std::fixed << std::setprecision(3) << v;
		return o.str();
	}

	std::string Key(const Sim::RunSpec& s) {
		return s.set + "|" + std::to_string(s.seed) + "|" + Sim::ConditionName(s.condition) + "|" + std::to_string(Sim::IsSingle(s.condition) ? 1 : s.n) +
		       "|" + std::to_string(std::lround(s.bitrate_bps)) + "|" + Fmt(s.repulsion_lead) + "|" + Fmt(s.ranging_noise);
	}

	// Runs already in the CSV (set, seed, condition, n), so an interrupted grid resumes.
	std::set<std::string> Done(const fs::path& csv) {
		std::set<std::string> keys;
		std::ifstream in(csv);
		std::string line;
		std::getline(in, line);  // header
		while (std::getline(in, line)) {
			std::vector<std::string> f;
			std::stringstream s(line);
			std::string x;
			while (std::getline(s, x, ',')) f.push_back(x);
			// The bitrate column (spec 13 §4) is absent in files from before WP9: no limit.
			const std::string bitrate = f.size() > 31 ? std::to_string(std::lround(std::stod(f[31]))) : "0";
			// The repulsion and ranging columns (spec 15 §5) are absent before WP11: off.
			const std::string lead = f.size() > 35 ? Fmt(std::stod(f[35])) : Fmt(0.0);
			const std::string ranging = f.size() > 36 ? Fmt(std::stod(f[36])) : Fmt(-1.0);
			if (f.size() > 6) keys.insert(f[0] + "|" + f[4] + "|" + f[5] + "|" + f[6] + "|" + bitrate + "|" + lead + "|" + ranging);
		}
		return keys;
	}

	int Run(int argc, char** argv) {
		fs::path ported, out = "results.csv", sets_out;
		fs::path examples = MRS_SPEC_EXAMPLES;
		std::vector<GenSpec> gens;
		std::vector<TreeSpec> tree_sets;
		std::vector<CondSpec> conds;
		std::vector<double> bitrates{0.0}, leads{0.0};
		double ranging = -1.0;
		int seed_a = 1, seed_b = 10, jobs = static_cast<int>(std::max(1u, std::thread::hardware_concurrency()));
		double limit = 4000.0;
		std::string commit = "unknown";
		for (int k = 2; k < argc; ++k) {
			const std::string a = argv[k];
			auto next = [&]() -> std::string {
				if (k + 1 >= argc) throw std::runtime_error("missing value after " + a);
				return argv[++k];
			};
			if (a == "--ported") ported = next();
			else if (a == "--out") out = next();
			else if (a == "--sets-out") sets_out = next();
			else if (a == "--examples") examples = next();
			else if (a == "--commit") commit = next();
			else if (a == "--jobs") jobs = std::stoi(next());
			else if (a == "--limit") limit = std::stod(next());
			else if (a == "--bitrate") {
				bitrates.clear();
				std::stringstream in(next());
				std::string part;
				while (std::getline(in, part, ',')) bitrates.push_back(std::stod(part));
			}
			else if (a == "--repulsion") {
				leads.clear();
				std::stringstream in(next());
				std::string part;
				while (std::getline(in, part, ',')) leads.push_back(std::stod(part));
			} else if (a == "--ranging") ranging = std::stod(next());
			else if (a == "--seeds") {
				const std::string v = next();
				const auto dash = v.find('-');
				seed_a = std::stoi(v.substr(0, dash));
				seed_b = dash == std::string::npos ? seed_a : std::stoi(v.substr(dash + 1));
			} else if (a == "--gen") {
				std::stringstream s(next());
				std::string f, d, n;
				std::getline(s, f, ':');
				std::getline(s, d, ':');
				std::getline(s, n, ':');
				GenSpec g{};
				if (!Sim::ParseFamily(f, g.family) || !Sim::ParseDispatch(d, g.dispatch)) throw std::runtime_error("bad --gen " + s.str());
				g.tasks = std::stoi(n);
				gens.push_back(g);
			} else if (a == "--trees") {
				std::stringstream s(next());
				std::string f, n, p, iv;
				std::getline(s, f, ':');
				std::getline(s, n, ':');
				std::getline(s, p, ':');
				std::getline(s, iv, ':');
				TreeSpec g{};
				if (!Sim::ParseTreeFamily(f, g.family) || n.empty() || p.empty()) throw std::runtime_error("bad --trees " + s.str());
				g.trees = std::stoi(n);
				g.parts = std::stoi(p);
				g.interval = iv.empty() ? 0.0 : std::stod(iv);
				tree_sets.push_back(g);
			} else if (a == "--cond") {
				const std::string v = next();
				const auto colon = v.find(':');
				CondSpec c{};
				if (!Sim::ParseCondition(v.substr(0, colon), c.condition)) throw std::runtime_error("bad --cond " + v);
				c.n = colon == std::string::npos ? std::vector<int>{1} : Ints(v.substr(colon + 1));
				conds.push_back(c);
			} else {
				throw std::runtime_error("unknown option " + a);
			}
		}
		if (conds.empty()) throw std::runtime_error("no --cond given");

		const Sim::RobotFiles files{ReadFile(examples / "mavic_webots.mrsd"), ReadFile(examples / "mavic_webots.mrsp"),
		                            (examples / "uav_behaviours.mrsb").string()};
		Sim::RunSpec base;
		base.time_limit = limit;
		base.mission.layer_step = 3.0;  // spec 10 §4

		// The run list: every set and seed under every condition.
		std::vector<Sim::RunSpec> runs;
		auto add = [&](Sim::RunSpec s) {
			s.ranging_noise = ranging;
			for (double lead : leads)
				for (double b : bitrates)
					for (const auto& c : conds)
						for (int n : c.n) {
							s.condition = c.condition;
							s.n = Sim::IsSingle(c.condition) ? 1 : n;
							s.bitrate_bps = b;
							s.repulsion_lead = lead;
							runs.push_back(s);
							if (Sim::IsSingle(c.condition)) break;
						}
		};
		if (!ported.empty()) {
			std::vector<fs::path> files_in;
			for (const auto& e : fs::directory_iterator(ported))
				if (e.path().extension() == ".mrsl") files_in.push_back(e.path());
			std::sort(files_in.begin(), files_in.end(), [](const fs::path& a, const fs::path& b) {
				const auto num = [](const fs::path& p) {
					const std::string s = p.stem().string();
					const auto at = s.find_first_of("0123456789");
					return at == std::string::npos ? 0 : std::stoi(s.substr(at));
				};
				return num(a) < num(b);
			});
			for (const auto& p : files_in) {
				Sim::RunSpec s = base;
				s.set = "ported/" + p.stem().string();
				s.family = "ported";
				s.dispatch = "even";
				s.timeline = ReadFile(p);
				for (int seed = seed_a; seed <= seed_b; ++seed) {
					s.seed = static_cast<std::uint64_t>(seed);
					add(s);
				}
			}
		}
		for (const auto& g : gens)
			for (int seed = seed_a; seed <= seed_b; ++seed) {
				Sim::GenConfig gc;
				gc.family = g.family;
				gc.dispatch = g.dispatch;
				gc.tasks = g.tasks;
				gc.seed = static_cast<std::uint64_t>(seed);
				Sim::RunSpec s = base;
				const std::string name = std::string(Sim::FamilyName(g.family)) + "-" + Sim::DispatchName(g.dispatch) + "-" + std::to_string(g.tasks);
				s.set = "gen/" + name + "/s" + std::to_string(seed);
				s.family = Sim::FamilyName(g.family);
				s.dispatch = Sim::DispatchName(g.dispatch);
				s.timeline = Sim::GenerateTaskSet(gc);
				s.seed = gc.seed;
				if (!sets_out.empty()) {
					fs::create_directories(sets_out / name);
					std::ofstream(sets_out / name / ("s" + std::to_string(seed) + ".mrsl"), std::ios::binary) << s.timeline;
				}
				add(s);
			}

		for (const auto& g : tree_sets)
			for (int seed = seed_a; seed <= seed_b; ++seed) {
				Sim::TreeGenConfig tc;
				tc.family = g.family;
				tc.trees = g.trees;
				tc.parts = g.parts;
				if (g.interval > 0) tc.interval = g.interval;
				tc.seed = static_cast<std::uint64_t>(seed);
				Sim::RunSpec s = base;
				const std::string name = std::string(Sim::TreeFamilyName(g.family)) + "-" + std::to_string(g.trees) + "x" + std::to_string(g.parts) +
				                         (g.interval > 0 ? "-i" + std::to_string(static_cast<int>(g.interval)) : std::string());
				s.set = "tree/" + name + "/s" + std::to_string(seed);
				s.family = Sim::TreeFamilyName(g.family);
				s.dispatch = "even";
				s.timeline = Sim::GenerateTreeSet(tc);
				s.seed = tc.seed;
				if (!sets_out.empty()) {
					fs::create_directories(sets_out / name);
					std::ofstream(sets_out / name / ("s" + std::to_string(seed) + ".mrsl"), std::ios::binary) << s.timeline;
				}
				add(s);
			}

		const auto done = Done(out);
		std::vector<Sim::RunSpec> todo;
		for (const auto& r : runs)
			if (!done.count(Key(r))) todo.push_back(r);
		const bool fresh = !fs::exists(out) || fs::file_size(out) == 0;
		std::ofstream csv(out, std::ios::app | std::ios::binary);
		if (fresh) csv << Sim::CsvHeader() << "\n";

		// Metadata next to the results (plan §10.4).
		{
			std::ofstream meta(out.string() + ".meta.txt", std::ios::app | std::ios::binary);
			const Sim::RunSpec& s = base;
			meta << "# mrs_experiment run\ncommit " << commit << "\nworld QuadSim (spec 08 §9)\nexamples " << examples.string()
			     << "\nmission " << s.mission.Text() << " (homes per run: a row " << s.home_spacing
			     << " m apart in seed order)\ngps_noise " << s.gps_noise << " m\nbattery " << s.battery_wh << " Wh\ntime_limit "
			     << s.time_limit << " s\nrta switch_margin " << s.rta.switch_margin << " claim_ttl " << s.rta.claim_ttl << "\nsta stack_bonus " << s.sta.stack_bonus << " relative_bonus " << s.sta.relative_bonus << "\nsize a1 "
			     << s.size.a1 << " a2 " << s.size.a2 << " a3 " << s.size.a3 << " a4 " << s.size.a4 << " pursuit " << s.size.pursuit
			     << "\ntravel v_xy " << s.travel.v_xy << " v_z " << s.travel.v_z << " settle " << s.travel.settle << " takeoff "
			     << s.travel.takeoff << " cruise_alt " << s.travel.cruise_alt
			     << "\nplanner S1/S1* objective makespan, G-C objective completion; improve_rounds 30\nruns " << runs.size() << " (" << todo.size() << " to do)\n\n";
		}

		std::mutex lock;
		std::atomic<std::size_t> next{0};
		std::atomic<int> finished{0};
		auto worker = [&]() {
			for (std::size_t i; (i = next++) < todo.size();) {
				Sim::RunResult r;
				try {
					r = Sim::RunOne(files, todo[i]);
				} catch (const std::exception& e) {
					std::lock_guard<std::mutex> g(lock);
					std::cerr << "run " << Key(todo[i]) << " failed: " << e.what() << "\n";
					continue;
				}
				std::lock_guard<std::mutex> g(lock);
				csv << Sim::CsvRow(todo[i], r) << "\n";
				csv.flush();
				const int f = ++finished;
				std::cerr << "[" << f << "/" << todo.size() << "] " << Key(todo[i]) << " makespan " << r.makespan
				          << (r.completed ? "" : " (stalled)") << " in " << r.wall_s << " s\n";
			}
		};
		std::vector<std::thread> pool;
		for (int k = 0; k < std::max(1, jobs); ++k) pool.emplace_back(worker);
		for (auto& t : pool) t.join();
		return 0;
	}
}

int main(int argc, char** argv) {
	try {
		const std::string cmd = argc > 1 ? argv[1] : "";
		if (cmd == "run") return Run(argc, argv);
		if (cmd == "gen" && argc == 6) {
			Sim::GenConfig c;
			if (!Sim::ParseFamily(argv[2], c.family) || !Sim::ParseDispatch(argv[3], c.dispatch)) throw std::runtime_error("bad family or dispatch");
			c.tasks = std::stoi(argv[4]);
			c.seed = std::stoull(argv[5]);
			std::cout << Sim::GenerateTaskSet(c);
			return 0;
		}
		if (cmd == "oracle" && argc == 3) {
			std::cout << Sim::OracleTimeline(ReadFile(argv[2]));
			return 0;
		}
		std::cerr << kUsage;
		return 2;
	} catch (const std::exception& e) {
		std::cerr << e.what() << "\n";
		return 1;
	}
}
