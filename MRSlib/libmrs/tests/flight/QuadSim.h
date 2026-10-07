#pragma once
// Rigid-body quadrotor platform for the WP4 tests (spec 08 §9). Not part of the library.
#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "mrs/port/Port.h"
#include "mrs/world/GeoReference.h"

namespace MRS {
	namespace Test {
		struct QuadParams {
			double mass = 0.55;
			double ixx = 0.010, iyy = 0.012, izz = 0.020;
			double kt = 0.00026, kq = 0.0000052;
			double drag = 0.15, ang_drag = 0.002;
			double com_x = -0.065;  // centre of mass, body x
			double p0 = 5.0, kp = 3.0e-5;  // battery power model: P = p0 + kp * sum |w|^3 (W)
			double capacity_wh = 50.0;
			double gps_noise = 0.0;  // m, uniform
		};

		class QuadSim : public Port::IPlatform {
		public:
			explicit QuadSim(QuadParams p = {}, Environment::GeoReference geo = Environment::GeoReference(56.9496, 24.1052, 10))
			    : p_(p), geo_(geo) {
				energy_wh_ = p_.capacity_wh;
				R_ = {1, 0, 0, 0, 1, 0, 0, 0, 1};
			}

			// --- state -------------------------------------------------------------------------
			std::array<double, 3> pos{0, 0, 0}, vel{0, 0, 0}, omega{0, 0, 0};
			std::array<double, 4> motor{0, 0, 0, 0};  // commanded speeds, signed
			std::map<std::string, double> leds;
			std::vector<std::string> sent;
			std::deque<std::string> inbox;
			bool crashed = false;
			double max_tilt_seen = 0.0;
			double energy_used_wh = 0.0;
			std::array<double, 3> wind{0, 0, 0};  // N, a constant disturbance force

			double Energy() const { return energy_wh_; }
			void SetEnergy(double wh) { energy_wh_ = wh; }
			void SetYaw(double yaw) {
				const double c = std::cos(yaw), s = std::sin(yaw);
				R_ = {c, -s, 0, s, c, 0, 0, 0, 1};
			}
			double Yaw() const { return std::atan2(R_[3], R_[0]); }
			double Roll() const { return std::atan2(R_[7], R_[8]); }
			double Pitch() const { return std::asin(std::clamp(-R_[6], -1.0, 1.0)); }
			bool OnGround() const { return pos[2] <= 1e-6; }

			// --- platform ----------------------------------------------------------------------
			std::vector<Port::PortInfo> Scan() override {
				std::vector<Port::PortInfo> out;
				for (const char* n : kDevices) out.push_back({Port::PortType::SIM, n, n});
				return out;
			}

			std::unique_ptr<Port::Port> Open(const Port::PortAssignment& a) override;

			double Time() const override { return t_; }
			bool Step() override {
				for (int k = 0; k < 8; ++k) Integrate(0.001);
				t_ += 0.008;
				++step_;
				return true;
			}
			long Steps() const { return step_; }
			const Environment::GeoReference& Geo() const { return geo_; }

		private:
			static constexpr const char* kDevices[] = {"front left propeller", "front right propeller", "rear left propeller",
			                                            "rear right propeller", "inertial unit", "gyro", "gps", "compass",
			                                            "front left led", "front right led", "battery", "emitter", "receiver"};
			// Motor positions from Mavic2Pro.proto: fl, fr, rl, rr; thrust constant signs.
			static constexpr double kMotorX[4] = {0.0548537, 0.0548537, -0.177179, -0.177179};
			static constexpr double kMotorY[4] = {0.151294, -0.151294, 0.127453, -0.127453};
			static constexpr double kSign[4] = {1, -1, -1, 1};

			void Integrate(double dt) {
				double thrust = 0, tx = 0, ty = 0, tz = 0, power = p_.p0;
				for (int i = 0; i < 4; ++i) {
					const double w = motor[i];
					const double T = kSign[i] * p_.kt * std::fabs(w) * w;
					thrust += T;
					tx += kMotorY[i] * T;
					ty -= (kMotorX[i] - p_.com_x) * T;
					tz -= p_.kq * std::fabs(w) * w;  // reaction torque, opposite to the spin
					power += p_.kp * std::fabs(w * w * w);
				}
				const double de = power * dt / 3600.0;
				energy_wh_ = std::max(0.0, energy_wh_ - de);
				energy_used_wh += de;

				// Forces in world frame: thrust along body z.
				double f[3] = {R_[2] * thrust - p_.drag * vel[0] + wind[0], R_[5] * thrust - p_.drag * vel[1] + wind[1],
				               R_[8] * thrust - p_.drag * vel[2] - p_.mass * 9.81 + wind[2]};
				const bool grounded = pos[2] <= 1e-6 && f[2] <= 0.0;
				if (grounded) {
					vel = {0, 0, 0};
					omega = {0, 0, 0};
					const double yaw = Yaw();
					SetYaw(yaw);
					pos[2] = 0;
					return;
				}
				for (int k = 0; k < 3; ++k) {
					vel[k] += f[k] / p_.mass * dt;
					pos[k] += vel[k] * dt;
				}
				if (pos[2] < 0) {
					if (vel[2] < -3.0) crashed = true;
					pos[2] = 0;
					vel[2] = 0;
				}
				// Rotation: Euler's equations in the body frame.
				const double I[3] = {p_.ixx, p_.iyy, p_.izz};
				const double tau[3] = {tx - p_.ang_drag * omega[0], ty - p_.ang_drag * omega[1], tz - p_.ang_drag * omega[2]};
				const double w0 = omega[0], w1 = omega[1], w2 = omega[2];
				omega[0] += (tau[0] - (I[2] - I[1]) * w1 * w2) / I[0] * dt;
				omega[1] += (tau[1] - (I[0] - I[2]) * w2 * w0) / I[1] * dt;
				omega[2] += (tau[2] - (I[1] - I[0]) * w0 * w1) / I[2] * dt;
				// R += R * skew(omega) * dt, then re-orthonormalise.
				const double S[9] = {0, -omega[2], omega[1], omega[2], 0, -omega[0], -omega[1], omega[0], 0};
				std::array<double, 9> n{};
				for (int r = 0; r < 3; ++r)
					for (int c = 0; c < 3; ++c) {
						double acc = 0;
						for (int k = 0; k < 3; ++k) acc += R_[r * 3 + k] * S[k * 3 + c];
						n[r * 3 + c] = R_[r * 3 + c] + acc * dt;
					}
				Orthonormalise(n);
				R_ = n;
				const double tilt = std::acos(std::clamp(R_[8], -1.0, 1.0));
				max_tilt_seen = std::max(max_tilt_seen, tilt);
			}

			static void Orthonormalise(std::array<double, 9>& m) {
				// Gram-Schmidt on the columns.
				double c0[3] = {m[0], m[3], m[6]}, c1[3] = {m[1], m[4], m[7]};
				auto norm = [](double* v) {
					const double l = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
					for (int k = 0; k < 3; ++k) v[k] /= l;
				};
				norm(c0);
				const double d = c0[0] * c1[0] + c0[1] * c1[1] + c0[2] * c1[2];
				for (int k = 0; k < 3; ++k) c1[k] -= d * c0[k];
				norm(c1);
				const double c2[3] = {c0[1] * c1[2] - c0[2] * c1[1], c0[2] * c1[0] - c0[0] * c1[2], c0[0] * c1[1] - c0[1] * c1[0]};
				m = {c0[0], c1[0], c2[0], c0[1], c1[1], c2[1], c0[2], c1[2], c2[2]};
			}

			double Noise() {
				seed_ = seed_ * 6364136223846793005ULL + 1442695040888963407ULL;
				return (static_cast<double>(seed_ >> 11) / 9007199254740992.0 * 2.0 - 1.0) * p_.gps_noise;
			}

			friend class QuadPort;
			QuadParams p_;
			Environment::GeoReference geo_;
			std::array<double, 9> R_;
			double energy_wh_ = 0;
			double t_ = 0;
			long step_ = 0;
			unsigned long long seed_ = 42;
		};

		class QuadPort : public ::MRS::Port::Port {
		public:
			QuadPort(::MRS::Port::PortAssignment a, QuadSim& sim) : ::MRS::Port::Port(std::move(a)), sim_(sim) {
				const double rate = Assignment().params.Num("rate_hz", 0.0);
				period_ = rate > 0 ? 1.0 / rate : 0.008;
			}
			bool Read(std::vector<double>& v) override {
				if (sim_.t_ + 1e-9 < next_) return false;
				next_ = sim_.t_ + period_ - 1e-9;
				const std::string& a = Assignment().address;
				const auto& R = sim_.R_;
				if (a == "gps") {
					double lat, lon, alt;
					sim_.geo_.ToGeodetic({sim_.pos[0] + sim_.Noise(), sim_.pos[1] + sim_.Noise(), sim_.pos[2] + sim_.Noise()}, lat, lon, alt);
					v = {lat, lon, alt};
				} else if (a == "inertial unit") {
					v = {sim_.Roll(), sim_.Pitch(), sim_.Yaw()};
				} else if (a == "gyro") {
					v = {sim_.omega[0], sim_.omega[1], sim_.omega[2]};
				} else if (a == "compass") {
					v = {R[1], R[4], R[7]};  // world north (0, 1, 0) in the body frame: column 1 of R
				} else if (a == "battery") {
					v = {sim_.energy_wh_};
				} else {
					return false;
				}
				return true;
			}
			bool Write(const std::vector<double>& v) override {
				const std::string& a = Assignment().address;
				static const char* motors[4] = {"front left propeller", "front right propeller", "rear left propeller",
				                                "rear right propeller"};
				for (int k = 0; k < 4; ++k)
					if (a == motors[k]) {
						sim_.motor[k] = v.at(0);
						return true;
					}
				sim_.leds[a] = v.at(0);
				return true;
			}
			bool Send(const std::string& data) override {
				sim_.sent.push_back(data);
				return true;
			}
			std::optional<std::string> Receive() override {
				if (sim_.inbox.empty()) return std::nullopt;
				auto m = sim_.inbox.front();
				sim_.inbox.pop_front();
				return m;
			}

		private:
			QuadSim& sim_;
			double period_;
			double next_ = 0;
		};

		inline std::unique_ptr<Port::Port> QuadSim::Open(const Port::PortAssignment& a) {
			for (const char* n : kDevices)
				if (a.address == n) return std::make_unique<QuadPort>(a, *this);
			return nullptr;
		}
	}
}
