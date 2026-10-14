#include "mrs/platform/Net.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <thread>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <cerrno>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace MRS {
	namespace Net {
		namespace {
#ifdef _WIN32
			using Native = SOCKET;
			const Native kInvalid = INVALID_SOCKET;
			using SockLen = int;
			void CloseNative(Native s) { closesocket(s); }
			bool SetNonBlocking(Native s) {
				u_long on = 1;
				return ioctlsocket(s, FIONBIO, &on) == 0;
			}
			bool WouldBlock() {
				const int e = WSAGetLastError();
				return e == WSAEWOULDBLOCK || e == WSAEINPROGRESS || e == WSAECONNRESET;  // ICMP port unreachable on UDP
			}
			struct WinsockInit {
				WinsockInit() {
					WSADATA data;
					WSAStartup(MAKEWORD(2, 2), &data);
				}
				~WinsockInit() { WSACleanup(); }
			};
			void EnsureInit() { static WinsockInit init; }
#else
			using Native = int;
			const Native kInvalid = -1;
			using SockLen = socklen_t;
			void CloseNative(Native s) { ::close(s); }
			bool SetNonBlocking(Native s) {
				const int flags = fcntl(s, F_GETFL, 0);
				return flags >= 0 && fcntl(s, F_SETFL, flags | O_NONBLOCK) == 0;
			}
			bool WouldBlock() { return errno == EAGAIN || errno == EWOULDBLOCK || errno == EINPROGRESS || errno == ECONNREFUSED; }
			void EnsureInit() {}
#endif

			Native ToNative(std::intptr_t fd) { return static_cast<Native>(fd); }
			std::intptr_t FromNative(Native s) { return s == kInvalid ? -1 : static_cast<std::intptr_t>(s); }

			bool Resolve(const Endpoint& e, sockaddr_in& out) {
				std::memset(&out, 0, sizeof(out));
				out.sin_family = AF_INET;
				out.sin_port = htons(static_cast<unsigned short>(e.port));
				if (inet_pton(AF_INET, e.host.c_str(), &out.sin_addr) == 1) return true;
				addrinfo hints{};
				hints.ai_family = AF_INET;
				addrinfo* res = nullptr;
				if (getaddrinfo(e.host.c_str(), nullptr, &hints, &res) != 0 || !res) return false;
				out.sin_addr = reinterpret_cast<sockaddr_in*>(res->ai_addr)->sin_addr;
				freeaddrinfo(res);
				return true;
			}

			Endpoint FromAddr(const sockaddr_in& a) {
				char buf[INET_ADDRSTRLEN] = {0};
				inet_ntop(AF_INET, &a.sin_addr, buf, sizeof(buf));
				return {buf, ntohs(a.sin_port)};
			}

			void WaitReadable(Native s, double seconds) {
				if (seconds <= 0.0) return;
				if (s == kInvalid) {
					// Nothing to wait on: sleep instead, so a caller's loop does not spin.
					std::this_thread::sleep_for(std::chrono::duration<double>(seconds));
					return;
				}
				fd_set set;
				FD_ZERO(&set);
				FD_SET(s, &set);
				timeval tv;
				tv.tv_sec = static_cast<long>(seconds);
				tv.tv_usec = static_cast<long>((seconds - static_cast<double>(tv.tv_sec)) * 1e6);
				select(static_cast<int>(s) + 1, &set, nullptr, nullptr, &tv);
			}

			class TcpLink : public IByteLink {
			public:
				explicit TcpLink(Endpoint to) : to_(std::move(to)) { Connect(); }
				~TcpLink() override { Close(); }

				bool Write(const std::uint8_t* data, std::size_t size) override {
					if (!Connected() && !Connect()) return false;
					std::size_t sent = 0;
					while (sent < size) {
						const auto n = ::send(s_, reinterpret_cast<const char*>(data + sent), static_cast<int>(size - sent), 0);
						if (n <= 0) {
							if (WouldBlock()) {
								WaitWritable();
								continue;
							}
							Close();
							return false;
						}
						sent += static_cast<std::size_t>(n);
					}
					return true;
				}
				std::size_t Read(std::uint8_t* data, std::size_t capacity) override {
					if (!Connected() && !Connect()) return 0;
					const auto n = ::recv(s_, reinterpret_cast<char*>(data), static_cast<int>(capacity), 0);
					if (n > 0) return static_cast<std::size_t>(n);
					if (n == 0 || !WouldBlock()) Close();  // closed by the other end
					return 0;
				}
				void Wait(double seconds) override { WaitReadable(s_, seconds); }
				bool Connected() const override { return s_ != kInvalid; }
				std::string Describe() const override { return "tcp:" + to_.host + ":" + std::to_string(to_.port); }

			private:
				bool Connect() {
					const auto now = std::chrono::steady_clock::now();
					if (tried_ && now - last_try_ < std::chrono::seconds(1)) return false;
					tried_ = true;
					last_try_ = now;
					sockaddr_in addr;
					if (!Resolve(to_, addr)) return false;
					Native s = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
					if (s == kInvalid) return false;
					if (::connect(s, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
						CloseNative(s);
						return false;
					}
					int one = 1;
					setsockopt(s, IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<const char*>(&one), sizeof(one));
					SetNonBlocking(s);
					s_ = s;
					return true;
				}
				void Close() {
					if (s_ != kInvalid) CloseNative(s_);
					s_ = kInvalid;
				}
				void WaitWritable() {
					fd_set set;
					FD_ZERO(&set);
					FD_SET(s_, &set);
					timeval tv{0, 10000};
					select(static_cast<int>(s_) + 1, nullptr, &set, nullptr, &tv);
				}

				Endpoint to_;
				Native s_ = kInvalid;
				bool tried_ = false;
				std::chrono::steady_clock::time_point last_try_{};
			};

			class UdpLink : public IByteLink {
			public:
				// listen: bind `port` and answer the last sender; otherwise send to `to`.
				UdpLink(bool listen, Endpoint where) : listen_(listen), where_(std::move(where)) {
					if (!socket_.Bind(listen_ ? where_.port : 0)) throw std::runtime_error("cannot bind UDP port " + std::to_string(where_.port));
					if (!listen_) peer_ = where_;
				}
				bool Write(const std::uint8_t* data, std::size_t size) override {
					if (!peer_) return false;
					return socket_.SendTo(*peer_, data, size);
				}
				std::size_t Read(std::uint8_t* data, std::size_t capacity) override {
					if (pending_.empty()) {
						Endpoint from;
						auto d = socket_.Receive(&from);
						if (!d) return 0;
						if (listen_) peer_ = from;
						pending_ = std::move(*d);
						at_ = 0;
					}
					const std::size_t n = std::min(capacity, pending_.size() - at_);
					std::memcpy(data, pending_.data() + at_, n);
					at_ += n;
					if (at_ >= pending_.size()) pending_.clear();
					return n;
				}
				void Wait(double seconds) override { socket_.Wait(seconds); }
				bool Connected() const override { return socket_.Ok() && peer_.has_value(); }
				std::string Describe() const override {
					return (listen_ ? "udpin:" : "udp:" + where_.host + ":") + std::to_string(where_.port);
				}

			private:
				bool listen_;
				Endpoint where_;
				UdpSocket socket_;
				std::optional<Endpoint> peer_;
				std::vector<std::uint8_t> pending_;
				std::size_t at_ = 0;
			};
		}

		Endpoint ParseEndpoint(const std::string& text) {
			const auto colon = text.rfind(':');
			Endpoint e;
			try {
				if (colon == std::string::npos) {
					e.host = "127.0.0.1";
					e.port = std::stoi(text);
				} else {
					e.host = text.substr(0, colon);
					e.port = std::stoi(text.substr(colon + 1));
				}
			} catch (const std::exception&) {
				throw std::runtime_error("bad endpoint '" + text + "' (HOST:PORT)");
			}
			if (e.port < 0 || e.port > 65535) throw std::runtime_error("bad port in '" + text + "'");
			return e;
		}

		std::unique_ptr<IByteLink> OpenLink(const std::string& url) {
			EnsureInit();
			const auto colon = url.find(':');
			if (colon == std::string::npos) throw std::runtime_error("link '" + url + "': use tcp:HOST:PORT, udpin:PORT or udp:HOST:PORT");
			const std::string kind = url.substr(0, colon), rest = url.substr(colon + 1);
			if (kind == "tcp") return std::make_unique<TcpLink>(ParseEndpoint(rest));
			if (kind == "udpin") return std::make_unique<UdpLink>(true, ParseEndpoint(rest));
			if (kind == "udp") return std::make_unique<UdpLink>(false, ParseEndpoint(rest));
			throw std::runtime_error("link '" + url + "': unknown kind " + kind);
		}

		// --- UdpSocket -------------------------------------------------------------------------

		UdpSocket::UdpSocket() {
			EnsureInit();
			fd_ = FromNative(::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP));
			if (fd_ >= 0) SetNonBlocking(ToNative(fd_));
		}

		UdpSocket::~UdpSocket() {
			if (fd_ >= 0) CloseNative(ToNative(fd_));
		}

		bool UdpSocket::Ok() const { return fd_ >= 0; }

		bool UdpSocket::Bind(int port, bool reuse, const std::string& address) {
			if (fd_ < 0) return false;
			if (reuse) {
				int one = 1;
				setsockopt(ToNative(fd_), SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char*>(&one), sizeof(one));
#ifdef SO_REUSEPORT
				setsockopt(ToNative(fd_), SOL_SOCKET, SO_REUSEPORT, reinterpret_cast<const char*>(&one), sizeof(one));
#endif
			}
			sockaddr_in addr;
			if (!Resolve({address, port}, addr)) return false;
			return ::bind(ToNative(fd_), reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0;
		}

		bool UdpSocket::JoinMulticast(const std::string& group) {
			ip_mreq m{};
			if (inet_pton(AF_INET, group.c_str(), &m.imr_multiaddr) != 1) return false;
			m.imr_interface.s_addr = htonl(INADDR_ANY);
			if (setsockopt(ToNative(fd_), IPPROTO_IP, IP_ADD_MEMBERSHIP, reinterpret_cast<const char*>(&m), sizeof(m)) != 0) return false;
			unsigned char loop = 1;
			setsockopt(ToNative(fd_), IPPROTO_IP, IP_MULTICAST_LOOP, reinterpret_cast<const char*>(&loop), sizeof(loop));
			return true;
		}

		bool UdpSocket::SendTo(const Endpoint& to, const std::uint8_t* data, std::size_t size) {
			if (fd_ < 0) return false;
			sockaddr_in addr;
			if (!Resolve(to, addr)) return false;
			const auto n = ::sendto(ToNative(fd_), reinterpret_cast<const char*>(data), static_cast<int>(size), 0,
			                        reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
			return n >= 0 && static_cast<std::size_t>(n) == size;
		}

		std::optional<std::vector<std::uint8_t>> UdpSocket::Receive(Endpoint* from) {
			if (fd_ < 0) return std::nullopt;
			std::vector<std::uint8_t> buf(65536);
			sockaddr_in addr{};
			SockLen len = sizeof(addr);
			const auto n = ::recvfrom(ToNative(fd_), reinterpret_cast<char*>(buf.data()), static_cast<int>(buf.size()), 0,
			                          reinterpret_cast<sockaddr*>(&addr), &len);
			if (n < 0) return std::nullopt;
			buf.resize(static_cast<std::size_t>(n));
			if (from) *from = FromAddr(addr);
			return buf;
		}

		void UdpSocket::Wait(double seconds) { WaitReadable(fd_ < 0 ? kInvalid : ToNative(fd_), seconds); }

		int UdpSocket::LocalPort() const {
			sockaddr_in addr{};
			SockLen len = sizeof(addr);
			if (fd_ < 0 || getsockname(ToNative(fd_), reinterpret_cast<sockaddr*>(&addr), &len) != 0) return 0;
			return ntohs(addr.sin_port);
		}
	}
}
