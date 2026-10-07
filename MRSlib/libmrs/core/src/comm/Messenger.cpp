#include "mrs/comm/Messenger.h"

#include <cmath>

#include "mrs/protocol/Parser.h"
#include "mrs/protocol/Writer.h"

namespace MRS {
	namespace Comm {
		using Protocol::Field;
		using Protocol::FieldType;

		std::string Encode(const Protocol::Record& message) {
			Protocol::WriteOptions o;
			o.compact = true;
			std::string text = Protocol::Write(message, o);
			while (!text.empty() && (text.back() == '\n' || text.back() == '\r')) text.pop_back();
			return text;
		}

		std::optional<Message> Decode(const std::string& text) {
			auto parsed = Protocol::Parse(text);
			if (!parsed.Ok() || parsed.document.records.size() != 1) return std::nullopt;
			const Protocol::Record& r = parsed.document.records[0];
			if (r.kind != 'M' || r.fields.size() < 6) return std::nullopt;
			Message m;
			m.code = r.code;
			m.ver = r.fields[0].n;
			m.mission = r.fields[1].s;
			m.sender = r.fields[2].s;
			m.recipient = r.fields[3].s;
			m.seq = r.fields[4].i;
			m.stamp = r.fields[5].n;
			m.record = r;
			return m;
		}

		Messenger::Messenger(ITransport& transport, std::string self) : transport_(transport), self_(std::move(self)) {}

		Protocol::Record Messenger::Begin(const std::string& code, const std::string& recipient, double t) {
			Protocol::Record r;
			r.kind = 'M';
			r.code = code;
			r.fields = {Field::MakeNum(0.1),
			            Field::MakeText(FieldType::Id, mission_.empty() ? std::string("none") : mission_),
			            Field::MakeText(FieldType::Id, self_),
			            Field::MakeText(FieldType::Id, recipient),
			            Field::MakeInt(++seq_),
			            Field::MakeNum(std::round(t * 1000.0) / 1000.0)};
			return r;
		}

		bool Messenger::Post(const Protocol::Record& message) {
			const std::string text = Encode(message);
			if (transport_.MaxPayload() != 0 && text.size() > transport_.MaxPayload()) {
				++stats_.dropped_size;
				return false;
			}
			if (!transport_.Send(text, message.fields.at(3).s)) return false;
			++stats_.sent;
			stats_.bytes_sent += static_cast<long>(text.size());
			return true;
		}

		std::vector<Message> Messenger::Receive() {
			std::vector<Message> out;
			for (const auto& text : transport_.Poll()) {
				auto m = Decode(text);
				if (!m) {
					++stats_.dropped_parse;
					continue;
				}
				if (std::floor(m->ver) != 0.0) {  // MAJOR version 0 only
					++stats_.dropped_version;
					continue;
				}
				if (m->sender == self_) {
					++stats_.own;
					continue;
				}
				if (!m->Broadcast() && m->recipient != self_) {
					++stats_.not_for_us;
					continue;
				}
				const bool mission_ok = mission_.empty() ? m->code == "M_MISSION" : m->mission == mission_;
				if (!mission_ok) {
					++stats_.dropped_mission;
					continue;
				}
				if (!seen_[m->sender].insert(m->seq).second) {
					++stats_.dropped_duplicate;
					continue;
				}
				++stats_.received;
				out.push_back(std::move(*m));
			}
			return out;
		}
	}
}
