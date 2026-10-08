#include "mrs/world/UavProcessors.h"

#include <cmath>
#include <map>

namespace MRS {
	namespace Environment {
		namespace {
			constexpr double kPi = 3.14159265358979323846;

			double Wrap2Pi(double a) {
				a = std::fmod(a, 2.0 * kPi);
				if (a < 0.0) a += 2.0 * kPi;
				if (a >= 2.0 * kPi) a -= 2.0 * kPi;
				return a;
			}

			// Stamp of a valid field or component, or a negative value.
			double StampOf(const Worldview& w, const std::string& path) {
				auto e = w.Raw(path);
				return e && e->valid ? e->stamp : -1.0;
			}

			double RawScalar(const Worldview& w, const std::string& path, double fallback) {
				auto e = w.Raw(path);
				if (!e || !e->valid || !std::holds_alternative<double>(e->value)) return fallback;
				return std::get<double>(e->value);
			}

			// Runs once per new stamp of an input field.
			class OnNewStamp {
			public:
				bool operator()(double stamp) {
					if (stamp < 0.0 || stamp <= last_) return false;
					last_ = stamp;
					return true;
				}

			private:
				double last_ = -1.0;
			};

			class ClockProcessor : public IViewProcessor {
			public:
				std::string Name() const override { return "Clock"; }
				std::vector<std::string> Subscriptions() const override { return {}; }
				std::vector<std::string> Provides() const override { return {"time"}; }
				void Tick(Worldview& w, double t) override { w.SetScalar("time", t, t, "clock"); }
			};

			class GnssProcessor : public IViewProcessor {
			public:
				std::string Name() const override { return "GnssProcessor"; }
				std::vector<std::string> Subscriptions() const override { return {"V_GEO"}; }
				std::vector<std::string> Provides() const override { return {"geo.position"}; }
				void Process(const View& v, Worldview& w, double) override {
					if (v.values.size() >= 3) w.SetComponents("geo.position", {"lat", "lon", "alt"}, v.values, v.stamp, "gnss");
				}
			};

			class GeoToLocalProcessor : public IViewProcessor {
			public:
				std::string Name() const override { return "GeoToLocalProcessor"; }
				std::vector<std::string> Subscriptions() const override { return {}; }
				std::vector<std::string> Needs() const override { return {"geo.position"}; }
				std::vector<Offer> Offers() const override { return {{"pose.enu", "gnss"}}; }
				void Tick(Worldview& w, double) override {
					const GeoReference* geo = w.Geo();
					if (!geo) return;  // no mission header yet
					const double stamp = StampOf(w, "geo.position.lat");
					if (!fresh_(stamp)) return;
					const Enu e = geo->ToEnu(RawScalar(w, "geo.position.lat", 0), RawScalar(w, "geo.position.lon", 0),
					                         RawScalar(w, "geo.position.alt", 0));
					w.OfferVec3("pose.enu", "gnss", e.x, e.y, e.z, stamp);
				}

			private:
				OnNewStamp fresh_;
			};

			class LocalPositionProcessor : public IViewProcessor {
			public:
				std::string Name() const override { return "LocalPositionProcessor"; }
				std::vector<std::string> Subscriptions() const override { return {"V_POS3"}; }
				std::vector<Offer> Offers() const override { return {{"pose.enu", "local"}}; }
				void Process(const View& v, Worldview& w, double) override {
					if (v.values.size() >= 3) w.OfferVec3("pose.enu", "local", v.values[0], v.values[1], v.values[2], v.stamp);
				}
			};

			class BaroAltitudeProcessor : public IViewProcessor {
			public:
				std::string Name() const override { return "BaroAltitudeProcessor"; }
				std::vector<std::string> Subscriptions() const override { return {"V_BARO"}; }
				std::vector<Offer> Offers() const override { return {{"alt.amsl", "baro"}}; }
				void Process(const View& v, Worldview& w, double) override {
					if (!v.values.empty()) w.OfferScalar("alt.amsl", "baro", v.values[0], v.stamp);
				}
			};

			class GnssAltitudeProcessor : public IViewProcessor {
			public:
				std::string Name() const override { return "GnssAltitudeProcessor"; }
				std::vector<std::string> Subscriptions() const override { return {}; }
				std::vector<std::string> Needs() const override { return {"geo.position"}; }
				std::vector<Offer> Offers() const override { return {{"alt.amsl", "gnss"}}; }
				void Tick(Worldview& w, double) override {
					const double stamp = StampOf(w, "geo.position.alt");
					if (fresh_(stamp)) w.OfferScalar("alt.amsl", "gnss", RawScalar(w, "geo.position.alt", 0), stamp);
				}

			private:
				OnNewStamp fresh_;
			};

			class LocalAltitudeProcessor : public IViewProcessor {
			public:
				std::string Name() const override { return "LocalAltitudeProcessor"; }
				std::vector<std::string> Subscriptions() const override { return {"V_POS3"}; }
				std::vector<Offer> Offers() const override { return {{"alt.amsl", "local"}}; }
				void Process(const View& v, Worldview& w, double) override {
					if (v.values.size() >= 3) w.OfferScalar("alt.amsl", "local", v.values[2] + w.GeoAltitude(), v.stamp);
				}
			};

			class RangeAltitudeProcessor : public IViewProcessor {
			public:
				std::string Name() const override { return "RangeAltitudeProcessor"; }
				std::vector<std::string> Subscriptions() const override { return {"V_RNG"}; }
				std::vector<Offer> Offers() const override { return {{"alt.agl", "range"}}; }
				void Process(const View& v, Worldview& w, double) override {
					if (!v.values.empty()) w.OfferScalar("alt.agl", "range", v.values[0], v.stamp);
				}
			};

			// payload.id: the package the robot carries, 0 when its bay is empty (spec 05 §5.3).
			class PayloadProcessor : public IViewProcessor {
			public:
				std::string Name() const override { return "PayloadProcessor"; }
				std::vector<std::string> Subscriptions() const override { return {"V_PAY"}; }
				std::vector<std::string> Provides() const override { return {"payload.id"}; }
				void Process(const View& v, Worldview& w, double) override {
					if (!v.values.empty()) w.SetScalar("payload.id", v.values[0], v.stamp, "payload");
				}
			};

			class AglFromAmslProcessor : public IViewProcessor {
			public:
				std::string Name() const override { return "AglFromAmslProcessor"; }
				std::vector<std::string> Subscriptions() const override { return {}; }
				std::vector<std::string> Needs() const override { return {"alt.amsl"}; }
				std::vector<Offer> Offers() const override { return {{"alt.agl", "amsl"}}; }
				void Tick(Worldview& w, double) override {
					const double stamp = StampOf(w, "alt.amsl");
					if (!fresh_(stamp)) return;
					// Flat ground at home (spec 05 §5.1).
					const double ground = w.GeoAltitude() + RawScalar(w, "home.enu.z", 0.0);
					w.OfferScalar("alt.agl", "amsl", RawScalar(w, "alt.amsl", 0) - ground, stamp);
				}

			private:
				OnNewStamp fresh_;
			};

			class AttitudeProcessor : public IViewProcessor {
			public:
				std::string Name() const override { return "AttitudeProcessor"; }
				std::vector<std::string> Subscriptions() const override { return {"V_ATT"}; }
				std::vector<std::string> Provides() const override { return {"att"}; }
				void Process(const View& v, Worldview& w, double) override {
					if (v.values.size() >= 3) w.SetComponents("att", {"roll", "pitch", "yaw"}, v.values, v.stamp, "imu");
				}
			};

			class RateProcessor : public IViewProcessor {
			public:
				std::string Name() const override { return "RateProcessor"; }
				std::vector<std::string> Subscriptions() const override { return {"V_RATE"}; }
				std::vector<std::string> Provides() const override { return {"rate.body"}; }
				void Process(const View& v, Worldview& w, double) override {
					if (v.values.size() >= 3) w.SetVec3("rate.body", v.values[0], v.values[1], v.values[2], v.stamp, "imu");
				}
			};

			class CompassHeadingProcessor : public IViewProcessor {
			public:
				std::string Name() const override { return "CompassHeadingProcessor"; }
				std::vector<std::string> Subscriptions() const override { return {"V_MAG"}; }
				std::vector<Offer> Offers() const override { return {{"heading", "compass"}}; }
				void Process(const View& v, Worldview& w, double) override {
					if (!v.values.empty()) w.OfferScalar("heading", "compass", Wrap2Pi(v.values[0]), v.stamp);
				}
			};

			class AttitudeHeadingProcessor : public IViewProcessor {
			public:
				std::string Name() const override { return "AttitudeHeadingProcessor"; }
				std::vector<std::string> Subscriptions() const override { return {"V_ATT"}; }
				std::vector<Offer> Offers() const override { return {{"heading", "attitude"}}; }
				void Process(const View& v, Worldview& w, double) override {
					if (v.values.size() >= 3) w.OfferScalar("heading", "attitude", HeadingFromYaw(v.values[2]), v.stamp);
				}
			};

			// Alpha-beta filter per axis over pose.enu (spec 05 §5.3).
			class KinematicsEstimator : public IViewProcessor {
			public:
				explicit KinematicsEstimator(const UavWorldviewConfig& c) : alpha_(c.alpha), beta_(c.beta), gamma_(c.gamma) {}
				std::string Name() const override { return "KinematicsEstimator"; }
				std::vector<std::string> Subscriptions() const override { return {}; }
				std::vector<std::string> Needs() const override { return {"pose.enu"}; }
				std::vector<std::string> Provides() const override { return {"vel.enu", "acc.enu"}; }
				void Tick(Worldview& w, double) override {
					const double stamp = StampOf(w, "pose.enu.x");
					if (stamp < 0.0 || stamp <= last_) return;
					const double z[3] = {RawScalar(w, "pose.enu.x", 0), RawScalar(w, "pose.enu.y", 0), RawScalar(w, "pose.enu.z", 0)};
					if (samples_ == 0) {
						for (int k = 0; k < 3; ++k) {
							x_[k] = z[k];
							v_[k] = 0.0;
						}
					} else {
						const double dt = stamp - last_;
						for (int k = 0; k < 3; ++k) {
							const double v_old = v_[k];
							if (samples_ == 1) {  // first velocity: plain difference
								v_[k] = (z[k] - x_[k]) / dt;
								x_[k] = z[k];
								continue;
							}
							const double predicted = x_[k] + v_[k] * dt;
							const double r = z[k] - predicted;
							x_[k] = predicted + alpha_ * r;
							v_[k] += beta_ * r / dt;
							const double a_raw = (v_[k] - v_old) / dt;
							a_[k] = samples_ == 2 ? a_raw : a_[k] + gamma_ * (a_raw - a_[k]);
						}
					}
					++samples_;
					last_ = stamp;
					if (samples_ >= 2) w.SetVec3("vel.enu", v_[0], v_[1], v_[2], stamp, "kinematics");
					if (samples_ >= 3) w.SetVec3("acc.enu", a_[0], a_[1], a_[2], stamp, "kinematics");
				}

			private:
				double alpha_, beta_, gamma_;
				double x_[3] = {0, 0, 0}, v_[3] = {0, 0, 0}, a_[3] = {0, 0, 0};
				int samples_ = 0;
				double last_ = -1.0;
			};

			class BatteryProcessor : public IViewProcessor {
			public:
				explicit BatteryProcessor(const UavWorldviewConfig& c) : low_(c.battery_low), critical_(c.battery_critical) {}
				std::string Name() const override { return "BatteryProcessor"; }
				std::vector<std::string> Subscriptions() const override { return {"V_BAT"}; }
				std::vector<std::string> Provides() const override { return {"battery", "battery.low", "battery.critical"}; }
				void Process(const View& v, Worldview& w, double) override {
					if (v.values.size() < 3) return;
					w.SetComponents("battery", {"voltage", "remaining", "energy_wh"}, v.values, v.stamp, "battery");
					w.SetBool("battery.low", v.values[1] < low_, v.stamp, "battery");
					w.SetBool("battery.critical", v.values[1] < critical_, v.stamp, "battery");
				}

			private:
				double low_, critical_;
			};

			class FlightStateProcessor : public IViewProcessor {
			public:
				explicit FlightStateProcessor(const UavWorldviewConfig& c) : c_(c) {}
				std::string Name() const override { return "FlightStateProcessor"; }
				std::vector<std::string> Subscriptions() const override { return {}; }
				std::vector<std::string> Needs() const override { return {"alt.agl", "vel.enu"}; }
				std::vector<std::string> Provides() const override { return {"airborne", "landed", "home"}; }
				void Tick(Worldview& w, double t) override {
					const auto agl = w.Scalar("alt.agl", t);
					const auto armed = w.Bool("armed", t);
					if (!agl) {
						w.Invalidate("airborne", t);
					} else {
						w.SetBool("airborne", *agl > c_.airborne_agl && armed.value_or(true), t, "flight_state");
					}

					const auto vx = w.Scalar("vel.enu.x", t), vy = w.Scalar("vel.enu.y", t), vz = w.Scalar("vel.enu.z", t);
					if (!agl || !vx || !vy || !vz) {
						w.Invalidate("landed", t);
						still_since_ = -1.0;
					} else {
						const double speed = std::sqrt(*vx * *vx + *vy * *vy + *vz * *vz);
						if (*agl < c_.landed_agl && speed < c_.landed_speed) {
							if (still_since_ < 0.0) still_since_ = t;
						} else {
							still_since_ = -1.0;
						}
						w.SetBool("landed", still_since_ >= 0.0 && t - still_since_ >= c_.landed_time - 1e-9, t, "flight_state");
					}

					const auto px = w.Scalar("pose.enu.x", t), py = w.Scalar("pose.enu.y", t);
					const auto hx = w.Raw("home.enu.x"), hy = w.Raw("home.enu.y");
					if (!px || !py || !hx || !hy) {
						w.Invalidate("home", t);
					} else {
						const double dx = *px - std::get<double>(hx->value), dy = *py - std::get<double>(hy->value);
						w.SetBool("home", std::sqrt(dx * dx + dy * dy) <= c_.home_radius, t, "flight_state");
					}
				}

			private:
				UavWorldviewConfig c_;
				double still_since_ = -1.0;
			};

			class GeofenceProcessor : public IViewProcessor {
			public:
				std::string Name() const override { return "GeofenceProcessor"; }
				std::vector<std::string> Subscriptions() const override { return {}; }
				std::vector<std::string> Needs() const override { return {"pose.enu"}; }
				std::vector<std::string> Provides() const override { return {"geofence.inside"}; }
				void Tick(Worldview& w, double t) override {
					const auto x = w.Scalar("pose.enu.x", t), y = w.Scalar("pose.enu.y", t), z = w.Scalar("pose.enu.z", t);
					double box[5];
					const char* keys[5] = {"geofence.xmin", "geofence.xmax", "geofence.ymin", "geofence.ymax", "geofence.zmax"};
					bool have_box = true;
					for (int k = 0; k < 5; ++k) {
						auto e = w.Raw(keys[k]);
						if (!e || !e->valid || !std::holds_alternative<double>(e->value)) have_box = false;
						else box[k] = std::get<double>(e->value);
					}
					if (!x || !y || !z || !have_box) {
						w.Invalidate("geofence.inside", t);
						return;
					}
					const bool inside = *x >= box[0] && *x <= box[1] && *y >= box[2] && *y <= box[3] && *z >= 0.0 && *z <= box[4];
					w.SetBool("geofence.inside", inside, t, "geofence");
				}
			};

			class PeerStateProcessor : public IViewProcessor {
			public:
				std::string Name() const override { return "PeerStateProcessor"; }
				std::vector<std::string> Subscriptions() const override { return {"V_PEER"}; }
				std::vector<std::string> Provides() const override { return {"peer"}; }
				void Process(const View& v, Worldview& w, double) override {
					if (v.values.size() < 8) return;
					const std::string id = "r" + std::to_string(static_cast<long long>(v.values[0]));
					const std::string base = "peer." + id;
					w.SetVec3(base + ".pose.enu", v.values[1], v.values[2], v.values[3], v.stamp, id);
					w.SetVec3(base + ".vel.enu", v.values[4], v.values[5], v.values[6], v.stamp, id);
					w.SetScalar(base + ".battery.remaining", v.values[7], v.stamp, id);
					w.SetId(base + ".task", v.text.empty() ? "0" : v.text, v.stamp, id);
					w.PutObject({id, "peer", {v.values[1], v.values[2], v.values[3]}, {v.values[4], v.values[5], v.values[6]}, v.stamp, id, 1.0});
				}
			};

			class DetectionProcessor : public IViewProcessor {
			public:
				explicit DetectionProcessor(const UavWorldviewConfig& c) : radius_(c.detection_radius) {}
				std::string Name() const override { return "DetectionProcessor"; }
				std::vector<std::string> Subscriptions() const override { return {"V_DET"}; }
				std::vector<std::string> Provides() const override { return {"det"}; }
				void Process(const View& v, Worldview& w, double) override {
					if (v.values.size() < 4 || v.text.empty()) return;
					const std::string prefix = "det." + v.text + ".";
					std::string id;
					int count = 0;
					for (const auto& o : w.Objects()) {
						if (o.second.cls != "detection" || o.first.compare(0, prefix.size(), prefix) != 0) continue;
						++count;
						const double dx = o.second.position.x - v.values[0], dy = o.second.position.y - v.values[1],
						             dz = o.second.position.z - v.values[2];
						if (id.empty() && std::sqrt(dx * dx + dy * dy + dz * dz) <= radius_) id = o.first;
					}
					if (id.empty()) id = prefix + std::to_string(count + 1);
					w.SetVec3(id + ".enu", v.values[0], v.values[1], v.values[2], v.stamp, "detector");
					w.SetScalar(id + ".confidence", v.values[3], v.stamp, "detector");
					w.PutObject({id, "detection", {v.values[0], v.values[1], v.values[2]}, {}, v.stamp, "detector", v.values[3]});
				}

			private:
				double radius_;
			};
		}

		double HeadingFromYaw(double yaw) { return Wrap2Pi(kPi / 2.0 - yaw); }

		std::vector<std::unique_ptr<IViewProcessor>> MakeUavProcessors(const UavWorldviewConfig& c) {
			std::vector<std::unique_ptr<IViewProcessor>> p;
			p.push_back(std::make_unique<ClockProcessor>());
			p.push_back(std::make_unique<GnssProcessor>());
			p.push_back(std::make_unique<GeoToLocalProcessor>());
			p.push_back(std::make_unique<LocalPositionProcessor>());
			p.push_back(std::make_unique<BaroAltitudeProcessor>());
			p.push_back(std::make_unique<GnssAltitudeProcessor>());
			p.push_back(std::make_unique<LocalAltitudeProcessor>());
			p.push_back(std::make_unique<RangeAltitudeProcessor>());
			p.push_back(std::make_unique<AglFromAmslProcessor>());
			p.push_back(std::make_unique<AttitudeProcessor>());
			p.push_back(std::make_unique<RateProcessor>());
			p.push_back(std::make_unique<CompassHeadingProcessor>());
			p.push_back(std::make_unique<AttitudeHeadingProcessor>());
			p.push_back(std::make_unique<KinematicsEstimator>(c));
			p.push_back(std::make_unique<BatteryProcessor>(c));
			p.push_back(std::make_unique<FlightStateProcessor>(c));
			p.push_back(std::make_unique<GeofenceProcessor>());
			p.push_back(std::make_unique<PeerStateProcessor>());
			p.push_back(std::make_unique<DetectionProcessor>(c));
			p.push_back(std::make_unique<PayloadProcessor>());
			return p;
		}

		void SetUavSourceOrders(Worldview& w) {
			w.SetSourceOrder("pose.enu", {"gnss", "local"});
			w.SetSourceOrder("alt.amsl", {"baro", "gnss", "local"});
			w.SetSourceOrder("alt.agl", {"range", "amsl"});
			w.SetSourceOrder("heading", {"compass", "attitude"});
		}

		void ApplyMissionHeader(const Protocol::Record& h, std::int64_t robot_id, Worldview& w, double t) {
			auto num = [&](std::size_t k) { return h.fields.at(k).n; };
			w.SetGeoReference(GeoReference(num(1), num(2), num(3)), num(3));
			const char* fence[5] = {"geofence.xmin", "geofence.xmax", "geofence.ymin", "geofence.ymax", "geofence.zmax"};
			for (std::size_t k = 0; k < 5; ++k) w.SetScalar(fence[k], num(4 + k), t, "mission");
			w.SetScalar("layer.alt", num(9) + static_cast<double>(robot_id - 1) * num(10), t, "mission");
			const auto n = static_cast<std::size_t>(h.fields.at(12).i);
			for (std::size_t k = 0; k < n; ++k) {
				const std::size_t at = 13 + 4 * k;
				if (h.fields.at(at).i == robot_id) w.SetVec3("home.enu", num(at + 1), num(at + 2), num(at + 3), t, "mission");
			}
		}

		WorldModel WorldModel::ForViews(const std::set<std::string>& views, const UavWorldviewConfig& config) {
			WorldModel m;
			for (auto& p : MakeUavProcessors(config)) m.chain.Add(std::move(p));
			m.chain.Prune(views);
			m.chain.Build();
			SetUavSourceOrders(m.world);
			return m;
		}

		void WorldModel::Update(const std::vector<View>& views, double t) {
			dt_ = last_t_ < 0.0 ? 0.0 : t - last_t_;
			last_t_ = t;
			chain.Update(views, world, t);
		}
	}
}
