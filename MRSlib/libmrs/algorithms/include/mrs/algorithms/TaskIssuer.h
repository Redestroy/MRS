#pragma once
// The operator side (plan §9, spec 09 §6): the task issuer that sends the mission header and
// dispatches a timeline, and the converter for the 2021 E-puck task sets.
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "mrs/comm/Messenger.h"
#include "mrs/protocol/Record.h"

namespace MRS {
	namespace Algorithms {
		struct IssuerConfig {
			std::string name = "op";
			double mission_period = 5.0;  // s between M_MISSION repeats, for robots that start late
			double task_period = 0.0;     // s between repeats of open tasks; 0: never
		};

		struct IssuedTask {
			std::string id;
			double dispatch = 0.0;              // when it was sent
			std::optional<double> done;         // first M_DONE
			std::string done_by;
			int done_count = 0;                 // M_DONE messages: more than one means duplicate arrivals
			std::optional<std::string> failed;  // M_FAIL reason
			std::vector<std::string> dumped_by; // robots that sent M_DUMP static
		};

		class TaskIssuer {
		public:
			TaskIssuer(Comm::ITransport& transport, Protocol::Record mission_header, IssuerConfig config = {});

			// Loads a timeline (.mrsl, spec 06 §7). Entries are sorted by time, file order kept
			// for equal times. Throws std::runtime_error on a parse error.
			void LoadTimeline(const std::string& text);
			// Schedules the top-level T records of a text at time t.
			void Schedule(double t, const std::vector<Protocol::Record>& tasks);

			// Sends the mission and the tasks that are due; reads results.
			void Tick(double t);
			// Every task was dispatched and has ended (done or failed).
			bool Done() const;
			// First dispatch to last completion, once Done.
			std::optional<double> Makespan() const;
			// Sends M_CMD cancel for a task.
			void Cancel(const std::string& task, double t);

			const std::map<std::string, IssuedTask>& Tasks() const { return issued_; }
			const Comm::Messenger& Messages() const { return messenger_; }

		private:
			struct Entry {
				double t;
				std::vector<Protocol::Record> tasks;
			};
			Comm::Messenger messenger_;
			Protocol::Record header_;
			IssuerConfig c_;
			std::vector<Entry> timeline_;
			std::size_t next_ = 0;
			double last_mission_ = -1e9, last_repeat_ = -1e9;
			std::map<std::string, IssuedTask> issued_;
			std::map<std::string, Protocol::Record> records_;
			std::optional<double> first_dispatch_;
		};

		// How a 2021 E-puck task set maps to a flying area (spec 09 §7).
		struct Port2021Config {
			double scale = 0.1;          // m per 2021 unit
			double cx = 500, cy = 500;   // 2021 point that becomes the ENU origin
			double altitude = 15.0;      // m
			double tol_xy = 1.0, tol_z = 0.5;
			double wait_scale = 0.1;     // s per 2021 A_W unit
			std::string issuer = "op";
		};

		// Converts a 2021 task set (one "<time> T: <id> <state> T_A <prio> /C_S C_P x y z tol/C_E
		// C_N/T_A n /A_L k/A_W w/..." line per task) to a timeline (.mrsl) text.
		std::string Port2021TaskSet(const std::string& text, const Port2021Config& config = {});
	}
}
