#pragma once
// Byte links and UDP sockets for the ArduPilot platform (spec 14 §2). POSIX and Winsock.
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace MRS {
	namespace Net {
		// A stream or packet link that carries MAVLink bytes. Reads never block.
		class IByteLink {
		public:
			virtual ~IByteLink() = default;
			virtual bool Write(const std::uint8_t* data, std::size_t size) = 0;
			// Up to `capacity` bytes that have arrived; 0 when none.
			virtual std::size_t Read(std::uint8_t* data, std::size_t capacity) = 0;
			// Blocks until bytes may be ready or `seconds` have passed.
			virtual void Wait(double seconds) = 0;
			virtual bool Connected() const = 0;
			virtual std::string Describe() const = 0;
		};

		// tcp:HOST:PORT   TCP client (ArduPilot SITL serial0 listens on tcp:127.0.0.1:5760); reconnects.
		// udpin:PORT      UDP, listening; replies go to the last sender (SITL --out or MAVProxy output).
		// udp:HOST:PORT   UDP, sending to HOST:PORT and reading its replies.
		// Throws std::runtime_error for a malformed url or a socket that cannot be made.
		std::unique_ptr<IByteLink> OpenLink(const std::string& url);

		struct Endpoint {
			std::string host;
			int port = 0;
		};
		// "HOST:PORT" or "PORT" (host 127.0.0.1).
		Endpoint ParseEndpoint(const std::string& text);

		// A non-blocking UDP socket.
		class UdpSocket {
		public:
			UdpSocket();
			~UdpSocket();
			UdpSocket(const UdpSocket&) = delete;
			UdpSocket& operator=(const UdpSocket&) = delete;

			// Binds to a local port (0: any). `reuse` lets several processes share it (multicast).
			bool Bind(int port, bool reuse = false, const std::string& address = "0.0.0.0");
			bool JoinMulticast(const std::string& group);
			bool SendTo(const Endpoint& to, const std::uint8_t* data, std::size_t size);
			// One datagram; nullopt when none is waiting. Fills `from` with the sender.
			std::optional<std::vector<std::uint8_t>> Receive(Endpoint* from = nullptr);
			void Wait(double seconds);
			int LocalPort() const;
			bool Ok() const;

		private:
			std::intptr_t fd_;
		};
	}
}
