#include "mrs/sim/Team.h"

#include <algorithm>

#include <cmath>
#include <regex>
#include <sstream>

#include "mrs/device/uav/UavDevices.h"
#include "mrs/protocol/Parser.h"
#include "mrs/task/BehaviourLibrary.h"
#include "mrs/task/Function.h"

namespace MRS {
	namespace Sim {
		// --- the radio channel -------------------------------------------------------------------

		bool Air::Link::Send(const std::string& text, const std::string&) {
			air_.outbox_.push_back(text);
			return true;
		}

		std::vector<std::string> Air::Link::Poll() {
			std::vector<std::string> out(inbox.begin(), inbox.end());
			inbox.clear();
			return out;
		}

		void Air::Send(const Queued& q, double t) {
			if (q.from != &op_) op_.inbox.push_back(q.text);
			for (auto* s : sims_)
				if (s != q.from) s->inbox.push_back(q.text);
			++messages;
			bytes += static_cast<long>(q.text.size());
			delay_sum += t - q.t;
			delay_max = std::max(delay_max, t - q.t);
		}

		void Air::Deliver(double t) {
			for (auto& text : outbox_) queue_.push_back({&op_, std::move(text), t});
			outbox_.clear();
			for (auto* s : sims_) {
				for (auto& text : s->sent) queue_.push_back({s, std::move(text), t});
				s->sent.clear();
			}
			if (bitrate_bps <= 0.0) {
				for (const auto& q : queue_) Send(q, t);
				queue_.clear();
				return;
			}
			// One shared channel, first come first served. Unused time is saved for 0.1 s, or for as
			// long as the message at the head needs.
			const double head = queue_.empty() ? 0.0 : static_cast<double>(queue_.front().text.size());
			credit_ = std::min(credit_ + bitrate_bps / 8.0 * std::max(0.0, t - last_t_), std::max(bitrate_bps / 8.0 * 0.1, head));
			last_t_ = t;
			while (!queue_.empty()) {
				Queued& q = queue_.front();
				// A message on the air is not dropped, however long it takes; one still waiting
				// behind others is, once it has waited max_delay.
				if (!q.sending && t - q.t > max_delay) {
					++dropped;
					queue_.pop_front();
					continue;
				}
				if (static_cast<double>(q.text.size()) > credit_) {
					q.sending = true;
					break;
				}
				credit_ -= static_cast<double>(q.text.size());
				Send(q, t);
				queue_.pop_front();
			}
		}

		// --- mission and robot files -------------------------------------------------------------

		std::string MissionSpec::Text() const {
			std::ostringstream s;
			s << "H: H_M " << id << " " << lat0 << " " << lon0 << " " << alt0 << " " << xmin << " " << xmax << " " << ymin << " "
			  << ymax << " " << zmax << " " << layer_base << " " << layer_step << " " << claim_grace << " " << homes.size();
			for (std::size_t k = 0; k < homes.size(); ++k) s << " " << (k + 1) << " " << homes[k][0] << " " << homes[k][1] << " " << homes[k][2];
			s << "/";
			return s.str();
		}

		Protocol::Record MissionSpec::Header() const {
			auto parsed = Protocol::Parse(Text());
			if (!parsed.Ok()) throw std::runtime_error("bad mission header: " + Text());
			return parsed.document.records.at(0);
		}

		std::vector<std::array<double, 3>> RowHomes(int n, double spacing, double y) {
			std::vector<std::array<double, 3>> homes;
			for (int k = 0; k < n; ++k) homes.push_back({(k - (n - 1) / 2.0) * spacing, y, 0.0});
			return homes;
		}

		std::string DefinitionFor(const std::string& base, int id, bool leds) {
			std::string d = std::regex_replace(base, std::regex(" id 1 "), " id " + std::to_string(id) + " ");
			if (!leds) {
				d = std::regex_replace(d, std::regex("D_4: D_A leds[^\n]*\n"), "");
				d = std::regex_replace(d, std::regex("K_1: K_A A_L[^\n]*\n"), "");
				d = std::regex_replace(d, std::regex("D_5: D_M"), "D_4: D_M");
				d = std::regex_replace(d, std::regex("D_6: D_C"), "D_5: D_C");
				d = std::regex_replace(d, std::regex("D_1\\.\\.6"), "D_1..5");
			}
			return d;
		}

		std::string PortMapFor(const std::string& base, bool leds) {
			return leds ? base : std::regex_replace(base, std::regex("P: P_A leds[^\n]*\n"), "");
		}

		// --- timing ------------------------------------------------------------------------------

		void TimedAllocator::Bind(const Algorithms::AllocatorContext& ctx) {
			IAllocator::Bind(ctx);
			inner_->Bind(ctx);
		}

		Algorithms::Decision TimedAllocator::Select(const Environment::Worldview& w, const Algorithms::CurrentTask& current, double t) {
			const auto start = std::chrono::steady_clock::now();
			auto d = inner_->Select(w, current, t);
			const double s = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
			++calls;
			total_s += s;
			worst_s = std::max(worst_s, s);
			return d;
		}

		// --- the team ----------------------------------------------------------------------------

		TeamRobot::TeamRobot(int id, const std::array<double, 3>& home, bool leds, const RobotFiles& files, const TeamConfig& c)
		    : sim([&] {
			      QuadParams p = c.quad;
			      p.seed += static_cast<unsigned long long>(id);
			      return p;
		      }(),
		          Environment::GeoReference(c.mission.lat0, c.mission.lon0, c.mission.alt0)) {
			sim.pos = home;
			Device::Uav::RegisterUavDevices(devices);
			Task::RegisterUavFunctions(functions);
			Task::TaskFactory factory(functions);
			Task::BehaviourLibrary library;
			library.PopulateFromFile(files.behaviours_path, factory);
			for (const auto& [name, priority] : c.behaviour_priority) library.SetPriority(name, priority);
			const auto map = Port::PortMap::Parse(PortMapFor(files.port_map, leds));
			robot = Device::BuildRobot(DefinitionFor(files.definition, id, leds), devices, sim, &map);
			Robot::ControllerConfig cc;
			cc.robot = "r" + std::to_string(id);
			cc.robot_id = id;
			ctl = std::make_unique<Robot::RobotController>(robot, library, functions, cc);
			transport = std::make_unique<Comm::CommBlockTransport>(robot.blocks.comms);
			auto inner = c.allocator ? c.allocator(id) : std::make_unique<Algorithms::RtaAllocator>();
			auto timed = std::make_unique<TimedAllocator>(std::move(inner));
			timing = timed.get();
			layer = std::make_unique<Algorithms::MrsLayer>(*ctl, *transport, functions, std::move(timed), [&c] {
				Algorithms::MrsConfig m = c.mrs;
				if (c.bitrate_bps > 0.0) m.bitrate_bps = c.bitrate_bps;
				return m;
			}());
		}

		Team::Team(const RobotFiles& files, const TeamConfig& c) : issuer(air.Operator(), c.mission.Header(), c.issuer), c_(c) {
			air.bitrate_bps = c.bitrate_bps;
			for (std::size_t k = 0; k < c.mission.homes.size(); ++k) {
				const int id = static_cast<int>(k + 1);
				robots.push_back(std::make_unique<TeamRobot>(id, c.mission.homes[k], !c.without_leds.count(id), files, c));
				air.Add(&robots.back()->sim);
			}
		}

		bool Team::Run(double seconds, bool stop_when_done) {
			const double end = Time() + seconds;
			while (Time() < end) {
				const double t = Time();
				issuer.Tick(t);
				for (auto& r : robots) r->layer->Tick(t);
				air.Deliver(t);
				if (stop_when_done && issuer.Done()) return true;
				for (auto& r : robots) r->sim.Step();
				Watch();
			}
			return stop_when_done ? issuer.Done() : false;
		}

		void Team::Watch() {
			++metrics.ticks;
			for (std::size_t i = 0; i < robots.size(); ++i) {
				const auto& p = robots[i]->sim.pos;
				const bool out = p[0] < c_.mission.xmin || p[0] > c_.mission.xmax || p[1] < c_.mission.ymin || p[1] > c_.mission.ymax ||
				                 p[2] > c_.mission.zmax;
				const int id = static_cast<int>(i);
				if (out && !outside_.count(id)) ++metrics.fence_exits;
				if (out) outside_.insert(id);
				else outside_.erase(id);
				if (p[2] < 0.5) continue;
				for (std::size_t j = i + 1; j < robots.size(); ++j) {
					const auto& q = robots[j]->sim.pos;
					if (q[2] < 0.5) continue;
					const double d = std::sqrt((p[0] - q[0]) * (p[0] - q[0]) + (p[1] - q[1]) * (p[1] - q[1]) + (p[2] - q[2]) * (p[2] - q[2]));
					metrics.min_separation = std::min(metrics.min_separation, d);
					const std::pair<int, int> pair{id, static_cast<int>(j)};
					if (d < c_.separation) {
						if (!close_.count(pair)) ++metrics.separation_breaches;
						close_.insert(pair);
					} else {
						close_.erase(pair);
					}
				}
			}
		}

		int Team::DoneBy(const std::string& name) const {
			int n = 0;
			for (const auto& [id, it] : issuer.Tasks()) n += it.done_by == name;
			return n;
		}
	}
}
