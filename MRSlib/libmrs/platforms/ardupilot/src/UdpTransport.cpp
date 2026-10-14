#include "mrs/platform/UdpTransport.h"

namespace MRS {
	namespace Net {
		UdpTransport::UdpTransport(UdpTransportConfig config) : c_(std::move(config)) {
			const bool group = !c_.multicast.empty();
			ok_ = socket_.Bind(c_.port, group) && (!group || socket_.JoinMulticast(c_.multicast));
			if (group) c_.peers.push_back({c_.multicast, c_.port});
		}

		bool UdpTransport::Send(const std::string& text, const std::string&) {
			if (!ok_ || text.size() > c_.max_payload) return false;
			bool all = true;
			for (const auto& p : c_.peers)
				all = socket_.SendTo(p, reinterpret_cast<const std::uint8_t*>(text.data()), text.size()) && all;
			return all;
		}

		std::vector<std::string> UdpTransport::Poll() {
			std::vector<std::string> out;
			while (auto d = socket_.Receive()) {
				std::string text(d->begin(), d->end());
				while (!text.empty() && (text.back() == '\0' || text.back() == '\n')) text.pop_back();
				if (!text.empty()) out.push_back(std::move(text));
			}
			return out;
		}
	}
}
