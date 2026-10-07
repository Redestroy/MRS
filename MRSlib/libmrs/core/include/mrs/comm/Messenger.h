#pragma once
// Messages between agents (spec 06 §2, §4; spec 09 §2): the envelope, transports and the
// messenger that encodes, sends, receives and filters them.
#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "mrs/device/BlockBuilder.h"
#include "mrs/protocol/Record.h"

namespace MRS {
	namespace Comm {
		// A decoded M record. Slot k is the k-th slot after the envelope.
		struct Message {
			std::string code;  // "M_CLAIM"
			double ver = 0.1;
			std::string mission, sender, recipient;
			std::int64_t seq = 0;
			double stamp = 0.0;
			Protocol::Record record;  // the whole M record

			const Protocol::Field& Slot(std::size_t k) const { return record.fields.at(6 + k); }
			std::size_t SlotCount() const { return record.fields.size() > 6 ? record.fields.size() - 6 : 0; }
			// The sub-record a Ref slot points to.
			const Protocol::Record& Child(std::size_t slot) const { return record.children.at(Slot(slot).ref); }
			bool Broadcast() const { return recipient == "all"; }
		};

		// Encodes and decodes the message form (one line, canonical, spec 06 §2).
		std::string Encode(const Protocol::Record& message);
		std::optional<Message> Decode(const std::string& text);

		// Where messages go. A broadcast-only link sends unicasts as broadcasts; the recipient
		// in the envelope tells the others to ignore them (spec 06 §2).
		class ITransport {
		public:
			virtual ~ITransport() = default;
			virtual bool Send(const std::string& text, const std::string& recipient) = 0;
			virtual std::vector<std::string> Poll() = 0;
			// The largest message, in bytes; 0 means no limit.
			virtual std::size_t MaxPayload() const { return 0; }
		};

		// A transport over the robot's communication devices (the radio on Webots).
		class CommBlockTransport : public ITransport {
		public:
			explicit CommBlockTransport(Device::CommBlock& block, std::size_t max_payload = 0) : block_(block), max_(max_payload) {}
			bool Send(const std::string& text, const std::string& recipient) override { return block_.Send(text, recipient); }
			std::vector<std::string> Poll() override { return block_.Receive(); }
			std::size_t MaxPayload() const override { return max_; }

		private:
			Device::CommBlock& block_;
			std::size_t max_;
		};

		struct MessengerStats {
			long sent = 0, received = 0, bytes_sent = 0;
			long dropped_parse = 0, dropped_version = 0, dropped_mission = 0, dropped_duplicate = 0, dropped_size = 0;
			long not_for_us = 0, own = 0;
		};

		class Messenger {
		public:
			Messenger(ITransport& transport, std::string self);

			// Messages of another mission are dropped; until a mission is set, only M_MISSION
			// passes (and sets nothing: the caller decides).
			void SetMission(std::string mission) { mission_ = std::move(mission); }
			const std::string& Mission() const { return mission_; }
			const std::string& Self() const { return self_; }

			// A new M record with the envelope filled in (the next sequence number, stamp t).
			Protocol::Record Begin(const std::string& code, const std::string& recipient, double t);
			// Encodes and sends. False when it does not fit the transport or the transport refuses it.
			bool Post(const Protocol::Record& message);

			// Every message received since the last call that is for this agent (spec 06 §2 rules).
			std::vector<Message> Receive();

			const MessengerStats& Stats() const { return stats_; }

		private:
			ITransport& transport_;
			std::string self_, mission_;
			std::int64_t seq_ = 0;
			std::map<std::string, std::set<std::int64_t>> seen_;
			MessengerStats stats_;
		};
	}
}
