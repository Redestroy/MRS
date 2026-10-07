#include "mrs/robot/TaskJournal.h"

#include <algorithm>
#include <cmath>

#include "mrs/protocol/Parser.h"
#include "mrs/protocol/Writer.h"

namespace MRS {
	namespace Robot {
		using Protocol::Field;
		using Protocol::FieldType;
		using Protocol::Record;

		void SetRecordState(Record& task, Task::TaskState state) {
			if (task.fields.size() > 2) task.fields[2] = Field::MakeInt(static_cast<std::int64_t>(state));
		}

		namespace {
			// Stamps to the millisecond, so the journal reads 4.36, not 4.360000000000003.
			Field Stamp(double t) { return Field::MakeNum(std::round(t * 1000.0) / 1000.0); }
		}

		void JournalWriter::Append(const Record& r) {
			if (!out_) return;
			*out_ << Protocol::Write(r);
			out_->flush();
		}

		void JournalWriter::Header(double stamp, const std::string& robot, const Record& mission_header) {
			Record r;
			r.kind = 'J';
			r.code = "J_H";
			r.fields = {Stamp(stamp), Field::MakeText(FieldType::Id, robot), Field::MakeRef(0)};
			r.children = {mission_header};
			Append(r);
		}

		void JournalWriter::Event(double stamp, const std::string& event) {
			Record r;
			r.kind = 'J';
			r.code = "J_E";
			r.fields = {Stamp(stamp), Field::MakeText(FieldType::Id, event)};
			Append(r);
		}

		void JournalWriter::TaskRecord(double stamp, const std::string& pool_state, std::size_t iterator, Record task,
		                               Task::TaskState state) {
			SetRecordState(task, state);
			Record r;
			r.kind = 'J';
			r.code = "J_T";
			r.fields = {Stamp(stamp), Field::MakeText(FieldType::Id, pool_state),
			            Field::MakeInt(static_cast<std::int64_t>(iterator)), Field::MakeRef(0)};
			r.children = {std::move(task)};
			Append(r);
		}

		void JournalWriter::Stack(double stamp, const std::vector<std::string>& ids) {
			Record r;
			r.kind = 'J';
			r.code = "J_K";
			r.fields = {Stamp(stamp), Field::MakeInt(static_cast<std::int64_t>(ids.size()))};
			for (const auto& id : ids) r.fields.push_back(Field::MakeText(FieldType::TaskId, id));
			Append(r);
		}

		std::vector<const JournalTask*> JournalState::Unfinished() const {
			std::vector<const JournalTask*> out;
			for (const auto& t : tasks)
				if (t.pool_state != "DONE" && t.pool_state != "FAILED") out.push_back(&t);
			return out;
		}

		namespace {
			// Cuts a partly written last record: keeps the text up to the last line that ends a
			// record ('/' as its last character outside a comment).
			std::string Complete(const std::string& text) {
				std::size_t keep = 0, pos = 0;
				while (pos < text.size()) {
					std::size_t end = text.find('\n', pos);
					const bool last = end == std::string::npos;
					if (last) end = text.size();
					std::string line = text.substr(pos, end - pos);
					const auto hash = line.find('#');
					if (hash != std::string::npos) line = line.substr(0, hash);
					while (!line.empty() && (line.back() == ' ' || line.back() == '\t' || line.back() == '\r')) line.pop_back();
					if (!line.empty() && line.back() == '/') keep = last ? end : end + 1;
					pos = end + 1;
				}
				return text.substr(0, keep);
			}
		}

		JournalState ReadJournal(const std::string& text) {
			JournalState s;
			auto parsed = Protocol::Parse(Complete(text));
			if (!parsed.Ok()) {
				s.error = std::string(Protocol::ErrorClassName(parsed.error->error_class)) + " at byte " +
				          std::to_string(parsed.error->offset) + ": " + parsed.error->message;
				return s;
			}
			for (const auto& r : parsed.document.records) {
				if (r.code == "J_H") {
					s.robot = r.fields[1].s;
					s.header = r.children.at(r.fields[2].ref);
					s.mission = s.header.fields.empty() ? std::string() : s.header.fields[0].s;
					s.ok = true;
				} else if (r.code == "J_T") {
					const Record& task = r.children.at(r.fields[3].ref);
					const std::string id = task.fields[0].s;
					auto it = std::find_if(s.tasks.begin(), s.tasks.end(), [&](const JournalTask& j) { return j.id == id; });
					if (it == s.tasks.end()) it = s.tasks.insert(s.tasks.end(), JournalTask{id, {}, 0, {}});
					it->pool_state = r.fields[1].s;
					it->iterator = static_cast<std::size_t>(std::max<std::int64_t>(0, r.fields[2].i));
					it->task = task;
				} else if (r.code == "J_K") {
					s.stack.clear();
					for (std::size_t k = 2; k < r.fields.size(); ++k) s.stack.push_back(r.fields[k].s);
				} else if (r.code == "J_E") {
					s.events.push_back(r.fields[1].s);
				}
			}
			if (!s.ok) s.error = "no J_H record";
			return s;
		}
	}
}
