#pragma once
// Messages between agents over UDP (spec 14 §6): Wi-Fi, a mesh, or one computer.
#include <memory>
#include <string>
#include <vector>

#include "mrs/comm/Messenger.h"
#include "mrs/platform/Net.h"

namespace MRS {
	namespace Net {
		struct UdpTransportConfig {
			int port = 14600;                 // local port the agent listens on
			std::vector<Endpoint> peers;      // every message goes to each of these
			std::string multicast;            // a group such as 239.255.77.1: join it and send to it on `port`
			std::size_t max_payload = 60000;  // bytes; one message is one datagram
		};

		// Broadcast semantics, as the Webots radio: every message goes to every peer, and the envelope's
		// recipient tells the others to ignore unicasts (spec 06 §2). With a multicast group, every agent
		// listens on the same port; otherwise each has its own port and lists the others as peers.
		class UdpTransport : public Comm::ITransport {
		public:
			explicit UdpTransport(UdpTransportConfig config);

			bool Send(const std::string& text, const std::string& recipient) override;
			std::vector<std::string> Poll() override;
			std::size_t MaxPayload() const override { return c_.max_payload; }

			bool Ok() const { return ok_; }
			// Blocks until a message may be waiting or `seconds` have passed.
			void Wait(double seconds) { socket_.Wait(seconds); }

		private:
			UdpTransportConfig c_;
			UdpSocket socket_;
			bool ok_ = false;
		};
	}
}
