#pragma once
// The task journal (spec 06 §8, spec 08 §7): an append-only record of a robot's tasks, so
// it can resume after a battery swap or a restart.
#include <map>
#include <ostream>
#include <string>
#include <vector>

#include "mrs/protocol/Record.h"
#include "mrs/task/Task.h"

namespace MRS {
	namespace Robot {
		class JournalWriter {
		public:
			explicit JournalWriter(std::ostream* out = nullptr) : out_(out) {}
			void SetStream(std::ostream* out) { out_ = out; }
			bool Enabled() const { return out_ != nullptr; }

			void Header(double stamp, const std::string& robot, const Protocol::Record& mission_header);
			void Event(double stamp, const std::string& event);  // start, swap_land, restart, end
			// `task` is the task's record; its state slot is set to `state`.
			void TaskRecord(double stamp, const std::string& pool_state, std::size_t iterator, Protocol::Record task,
			                Task::TaskState state);
			void Stack(double stamp, const std::vector<std::string>& ids);

		private:
			void Append(const Protocol::Record& r);
			std::ostream* out_;
		};

		// What a journal says, after the rules of spec 06 §8.
		struct JournalTask {
			std::string id;
			std::string pool_state;
			std::size_t iterator = 0;
			Protocol::Record task;  // the last J_T record's task
		};

		struct JournalState {
			bool ok = false;           // a J_H was found
			std::string error;         // why not
			std::string robot;
			std::string mission;       // H_M mission id
			Protocol::Record header;   // the H_M record
			std::vector<JournalTask> tasks;  // last J_T per task id, in first-seen order
			std::vector<std::string> stack;  // last J_K, bottom to top
			std::vector<std::string> events; // J_E events in order

			// Tasks whose last state is neither DONE nor FAILED.
			std::vector<const JournalTask*> Unfinished() const;
		};

		// Reads a journal; a partly written last record is ignored.
		JournalState ReadJournal(const std::string& text);

		// Sets the state slot (slot 3) of a task record.
		void SetRecordState(Protocol::Record& task, Task::TaskState state);
	}
}
