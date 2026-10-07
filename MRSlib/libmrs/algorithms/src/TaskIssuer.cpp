#include "mrs/algorithms/TaskIssuer.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include "mrs/protocol/Parser.h"

namespace MRS {
	namespace Algorithms {
		using Protocol::Field;
		using Protocol::FieldType;
		using Protocol::Record;

		TaskIssuer::TaskIssuer(Comm::ITransport& transport, Record mission_header, IssuerConfig config)
		    : messenger_(transport, config.name), header_(std::move(mission_header)), c_(std::move(config)) {
			messenger_.SetMission(header_.fields.at(0).s);
		}

		void TaskIssuer::LoadTimeline(const std::string& text) {
			auto parsed = Protocol::Parse(text);
			if (!parsed.Ok())
				throw std::runtime_error(std::string(Protocol::ErrorClassName(parsed.error->error_class)) + " at byte " +
				                         std::to_string(parsed.error->offset) + ": " + parsed.error->message);
			for (const auto& r : parsed.document.records) {
				if (r.code != "L_D") continue;
				std::vector<Record> tasks;
				for (std::size_t k = 1; k < r.fields.size(); ++k)
					if (r.fields[k].type == FieldType::Ref) tasks.push_back(r.children.at(r.fields[k].ref));
				Schedule(r.fields[0].n, tasks);
			}
		}

		void TaskIssuer::Schedule(double t, const std::vector<Record>& tasks) {
			// Sorted by time, file order kept for equal times (fixes the 2021 supervisor).
			auto it = std::upper_bound(timeline_.begin() + static_cast<long>(next_), timeline_.end(), t,
			                           [](double v, const Entry& e) { return v < e.t; });
			timeline_.insert(it, Entry{t, tasks});
		}

		void TaskIssuer::Tick(double t) {
			if (t - last_mission_ >= c_.mission_period) {
				last_mission_ = t;
				auto r = messenger_.Begin("M_MISSION", "all", t);
				r.fields.push_back(Field::MakeRef(0));
				r.children.push_back(header_);
				messenger_.Post(r);
			}
			while (next_ < timeline_.size() && timeline_[next_].t <= t) {
				const Entry& e = timeline_[next_++];
				// One message per task, so each fits the link (spec 06 §2).
				for (const auto& task : e.tasks) {
					auto r = messenger_.Begin("M_TASK", "all", t);
					r.fields.push_back(Field::MakeRef(0));
					r.children.push_back(task);
					messenger_.Post(r);
					const std::string id = task.fields.at(0).s;
					records_[id] = task;
					IssuedTask& it = issued_[id];
					it.id = id;
					it.dispatch = t;
					if (!first_dispatch_) first_dispatch_ = t;
				}
			}
			if (c_.task_period > 0 && t - last_repeat_ >= c_.task_period) {
				last_repeat_ = t;
				for (const auto& [id, it] : issued_) {
					if (it.done || it.failed) continue;
					auto r = messenger_.Begin("M_TASK", "all", t);
					r.fields.push_back(Field::MakeRef(0));
					r.children.push_back(records_.at(id));
					messenger_.Post(r);
				}
			}
			for (const auto& m : messenger_.Receive()) {
				if (m.SlotCount() == 0 || m.Slot(0).type != FieldType::TaskId) continue;
				auto it = issued_.find(m.Slot(0).s);
				if (it == issued_.end()) continue;
				if (m.code == "M_DONE") {
					++it->second.done_count;
					if (!it->second.done) {
						it->second.done = m.stamp;
						it->second.done_by = m.sender;
					}
				} else if (m.code == "M_FAIL") {
					if (!it->second.failed && !it->second.done) it->second.failed = m.Slot(1).s;
				} else if (m.code == "M_DUMP" && m.Slot(2).b) {
					it->second.dumped_by.push_back(m.sender);
				}
			}
		}

		bool TaskIssuer::Done() const {
			if (next_ < timeline_.size()) return false;
			for (const auto& [id, it] : issued_)
				if (!it.done && !it.failed) return false;
			return true;
		}

		std::optional<double> TaskIssuer::Makespan() const {
			if (!Done() || !first_dispatch_) return std::nullopt;
			double last = *first_dispatch_;
			for (const auto& [id, it] : issued_)
				if (it.done) last = std::max(last, *it.done);
			return last - *first_dispatch_;
		}

		void TaskIssuer::Cancel(const std::string& task, double t) {
			auto r = messenger_.Begin("M_CMD", "all", t);
			r.fields.push_back(Field::MakeText(FieldType::Id, "cancel"));
			r.fields.push_back(Field::MakeText(FieldType::TaskId, task));
			messenger_.Post(r);
			if (auto it = issued_.find(task); it != issued_.end() && !it->second.done) it->second.failed = "CANCELLED";
		}

		// --- the 2021 task sets ------------------------------------------------------------------

		namespace {
			std::string Num(double v) {
				std::ostringstream s;
				s << std::setprecision(6) << (std::fabs(v) < 5e-7 ? 0.0 : v);
				return s.str();
			}

			// An LED colour per 2021 LED index, so tasks are told apart in Webots.
			long Colour(long k) {
				static const long kColours[] = {0xFF0000, 0x00FF00, 0x0000FF, 0xFFFF00, 0x00FFFF, 0xFF00FF, 0xFFFFFF, 0xFF8000};
				return kColours[((k % 8) + 8) % 8];
			}
		}

		std::string Port2021TaskSet(const std::string& text, const Port2021Config& c) {
			std::ostringstream out;
			out << "# Ported from a 2021 E-puck task set (spec 09 §7): " << c.scale << " m per unit, origin at (" << c.cx << ", "
			    << c.cy << "), altitude " << c.altitude << " m.\n@: MRS 0.1/\n";
			std::istringstream lines(text);
			std::string line;
			while (std::getline(lines, line)) {
				if (line.find("T:") == std::string::npos) continue;
				std::istringstream s(line);
				double time = 0;
				std::string tk, kind;
				long id = 0, state = 0;
				double prio = 0;
				s >> time >> tk >> id >> state >> kind >> prio;
				const auto cp = line.find("C_P");
				if (!s || cp == std::string::npos) throw std::runtime_error("not a 2021 task line: " + line);
				double x = 0, y = 0, z = 0, tol = 0;
				std::istringstream p(line.substr(cp + 3));
				p >> x >> y >> z >> tol;
				std::vector<std::string> actions;
				bool led_on = false;
				for (std::size_t at = line.find("/A_"); at != std::string::npos; at = line.find("/A_", at + 1)) {
					std::istringstream a(line.substr(at + 1));
					std::string code;
					double v = 0;
					a >> code >> v;
					if (code == "A_L") {
						led_on = !led_on;  // 2021 sets toggled the LED: on, wait, off, wait
						actions.push_back("A_L 3 " + std::to_string(led_on ? Colour(static_cast<long>(v)) : 0));
					} else if (code == "A_W") {
						actions.push_back("A_W " + Num(v * c.wait_scale));
					}
				}
				actions.push_back("A_N");
				const double ex = (x - c.cx) * c.scale, ey = (y - c.cy) * c.scale;
				out << "\n# 2021 task " << id << " at (" << Num(x) << ", " << Num(y) << ")\n";
				out << "L: L_D " << Num(time) << " T_1/\n";
				out << "T_1: T_A " << c.issuer << "." << (id + 1) << " " << Num(prio) << " 0 C_1 C_2 A_1";
				if (actions.size() > 1) out << ".." << actions.size();
				out << "/\nC_1: C_P3 " << Num(ex) << " " << Num(ey) << " " << Num(c.altitude) << " " << Num(c.tol_xy) << " "
				    << Num(c.tol_z) << " 0 -1/\nC_2: C_N/\n";
				for (std::size_t k = 0; k < actions.size(); ++k) out << "A_" << (k + 1) << ": " << actions[k] << "/\n";
			}
			return out.str();
		}
	}
}
