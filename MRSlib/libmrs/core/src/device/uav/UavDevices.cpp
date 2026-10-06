#include "mrs/device/uav/UavDevices.h"

#include <cmath>

#include "mrs/BuildError.h"

namespace MRS {
	namespace Device {
		namespace Uav {
			namespace {
				constexpr double kPi = 3.14159265358979323846;

				// The port type of a device: parameter `port`, SIM by default.
				Port::PortType PortTypeOf(const Params& p) {
					const std::string name = p.Text("port", "SIM");
					auto type = Port::ParsePortType(name);
					if (!type) throw BuildError("unknown port type " + name);
					return *type;
				}

				std::string Required(const Params& p, const std::string& key) {
					std::string v = p.Text(key);
					if (v.empty()) throw BuildError("parameter " + key + " is required");
					return v;
				}

				// Copies the keys that exist from one parameter list to another.
				void Pass(const Params& from, Params& to, std::initializer_list<const char*> keys) {
					for (const char* key : keys)
						for (const ParamValue* v : from.All(key)) to.Add(key, *v);
				}
			}

			// --- RotorMotor ------------------------------------------------------------------

			void RotorMotor::OnConfigure() {
				std::map<std::string, double> limits;
				if (GetParams().Has("max_speed")) limits["max"] = GetParams().Num("max_speed", 0.0);
				Declare(Capability::Act("A_MOT", limits));
				DeclarePort("motor", PortTypeOf(GetParams()), Required(GetParams(), "device"), true, RateParams());
			}

			ActionStatus RotorMotor::Apply(const Action& action, double) {
				if (action.code != "A_MOT") return ActionStatus::REJECTED;
				const double speed = UnpackReals(ArgLayout::F64, action.arg).at(0);
				Port::Port* port = GetPort("motor");
				if (!port || !port->Write({speed})) return ActionStatus::FAILED;
				speed_ = speed;
				return ActionStatus::DONE;
			}

			// --- FlightControlUnit -----------------------------------------------------------

			void FlightControlUnit::OnConfigure() {
				const double climb = GetParams().Num("max_climb", 2.0);
				const double speed = GetParams().Num("max_speed_xy", 8.0);
				for (const char* code : {"A_TO", "A_LD", "A_HD", "A_PXY", "A_PZY", "A_VXY", "A_VZY"})
					Declare(Capability::Act(code, {{"max_climb", climb}, {"max_speed_xy", speed}}));
			}

			std::vector<const DeviceNode*> FlightControlUnit::DependsOn() const {
				std::vector<const DeviceNode*> deps;
				for (RotorMotor* m : motors_)
					if (m) deps.push_back(m);
				return deps;
			}

			ActionStatus FlightControlUnit::Apply(const Action& action, double t) {
				if (!HasCapability(CapabilityKind::Action, action.code)) return ActionStatus::REJECTED;
				setpoints_[action.code] = UnpackReals(ArgLayout::F32X2, action.arg);
				last_code_ = action.code;
				if (action.code == "A_HD") {
					const double duration = setpoints_[action.code][0];
					if (duration <= 0.0) return ActionStatus::DONE;
					if (hold_until_ < 0.0 || hold_arg_ != action.arg) {
						hold_until_ = t + duration;
						hold_arg_ = action.arg;
					}
					if (t + 1e-9 < hold_until_) return ActionStatus::RUNNING;
					hold_until_ = -1.0;
					return ActionStatus::DONE;
				}
				hold_until_ = -1.0;
				// Takeoff and landing finish when the flight controller (WP4) says so.
				if (action.code == "A_TO" || action.code == "A_LD") return ActionStatus::RUNNING;
				return ActionStatus::DONE;  // a setpoint is taken at once
			}

			// --- Sensors ---------------------------------------------------------------------

			void Imu::OnConfigure() {
				const auto type = PortTypeOf(GetParams());
				Declare(Capability::ViewOf("V_ATT"));
				Declare(Capability::ViewOf("V_RATE"));
				DeclarePort("inertial", type, Required(GetParams(), "inertial"), true, RateParams());
				DeclarePort("gyro", type, Required(GetParams(), "gyro"), true, RateParams());
				if (GetParams().Has("accelerometer")) {
					Declare(Capability::ViewOf("V_ACC"));
					DeclarePort("accelerometer", type, GetParams().Text("accelerometer"), true, RateParams());
				}
			}

			void Imu::Sample(double t, std::vector<View>& out) {
				static const std::pair<const char*, const char*> kPorts[] = {
					{"inertial", "V_ATT"}, {"gyro", "V_RATE"}, {"accelerometer", "V_ACC"}};
				std::vector<double> v;
				for (const auto& p : kPorts) {
					Port::Port* port = GetPort(p.first);
					if (port && port->Read(v) && v.size() >= 3) out.push_back({p.second, t, {v[0], v[1], v[2]}, {}});
				}
			}

			void Gnss::OnConfigure() {
				const std::string frame = GetParams().Text("frame", "wgs84");
				if (frame != "wgs84" && frame != "local") throw BuildError("frame must be wgs84 or local");
				local_ = frame == "local";
				Declare(Capability::ViewOf(local_ ? "V_POS3" : "V_GEO"));
				Params port = RateParams();
				port.Add("frame", IdValue(frame));  // a platform can check its coordinate system
				DeclarePort("gnss", PortTypeOf(GetParams()), Required(GetParams(), "device"), true, port);
			}

			void Gnss::Sample(double t, std::vector<View>& out) {
				std::vector<double> v;
				Port::Port* port = GetPort("gnss");
				if (port && port->Read(v) && v.size() >= 3) out.push_back({local_ ? "V_POS3" : "V_GEO", t, {v[0], v[1], v[2]}, {}});
			}

			double Compass::Heading(double north_x, double north_y) {
				double h = std::atan2(north_y, north_x);
				if (h < 0.0) h += 2.0 * kPi;
				if (h >= 2.0 * kPi) h -= 2.0 * kPi;
				return h;
			}

			void Compass::OnConfigure() {
				Declare(Capability::ViewOf("V_MAG"));
				DeclarePort("compass", PortTypeOf(GetParams()), Required(GetParams(), "device"), true, RateParams());
			}

			void Compass::Sample(double t, std::vector<View>& out) {
				std::vector<double> v;
				Port::Port* port = GetPort("compass");
				if (port && port->Read(v) && v.size() >= 2) out.push_back({"V_MAG", t, {Heading(v[0], v[1])}, {}});
			}

			void Barometer::OnConfigure() {
				Declare(Capability::ViewOf("V_BARO"));
				DeclarePort("baro", PortTypeOf(GetParams()), Required(GetParams(), "device"), true, RateParams());
			}

			void Barometer::Sample(double t, std::vector<View>& out) {
				std::vector<double> v;
				Port::Port* port = GetPort("baro");
				if (port && port->Read(v) && !v.empty()) out.push_back({"V_BARO", t, {v[0]}, {}});
			}

			// --- LedArray --------------------------------------------------------------------

			void LedArray::OnConfigure() {
				const auto devices = GetParams().Texts("device");
				if (devices.empty()) throw BuildError("an LED array needs at least one device");
				if (devices.size() > 32) throw BuildError("an LED array has at most 32 LEDs (A_L mask is 32 bits)");
				std::map<std::string, double> limits;
				if (GetParams().Has("rate_hz")) limits["rate_hz"] = GetParams().Num("rate_hz", 0.0);
				Declare(Capability::Act("A_L", limits));
				const auto type = PortTypeOf(GetParams());
				count_ = devices.size();
				Params port;
				if (GetParams().Has("rgb")) port.Add("rgb", BoolValue(GetParams().Bool("rgb", false)));
				for (std::size_t k = 0; k < devices.size(); ++k) DeclarePort("led" + std::to_string(k), type, devices[k], true, port);
			}

			ActionStatus LedArray::Apply(const Action& action, double) {
				if (action.code != "A_L") return ActionStatus::REJECTED;
				const auto halves = UnpackIntegers(ArgLayout::U32X2, action.arg);
				const auto mask = static_cast<std::uint32_t>(halves.at(0));
				const auto colour = static_cast<double>(halves.at(1));
				bool ok = true;
				for (std::size_t k = 0; k < count_; ++k) {
					Port::Port* port = GetPort("led" + std::to_string(k));
					ok = port && port->Write({(mask >> k) & 1u ? colour : 0.0}) && ok;
				}
				return ok ? ActionStatus::DONE : ActionStatus::FAILED;
			}

			// --- Battery ---------------------------------------------------------------------

			void Battery::OnConfigure() {
				capacity_wh_ = GetParams().Num("capacity_wh", 0.0);
				if (capacity_wh_ <= 0.0) throw BuildError("capacity_wh must be > 0");
				voltage_ = GetParams().Num("voltage", 11.55);
				Declare(Capability::Store("energy_wh", capacity_wh_));
				Declare(Capability::ViewOf("V_BAT"));
				DeclarePort("battery", PortTypeOf(GetParams()), GetParams().Text("device", "battery"), true, RateParams());
			}

			void Battery::Sample(double t, std::vector<View>& out) {
				std::vector<double> v;
				Port::Port* port = GetPort("battery");
				if (!port || !port->Read(v) || v.empty()) return;
				const double energy = v[0];
				double remaining = energy / capacity_wh_;
				if (remaining < 0.0) remaining = 0.0;
				if (remaining > 1.0) remaining = 1.0;
				out.push_back({"V_BAT", t, {v.size() > 1 ? v[1] : voltage_, remaining, energy}, {}});
			}

			// --- Radio -----------------------------------------------------------------------

			void Radio::OnConfigure() {
				std::map<std::string, double> limits;
				if (GetParams().Has("range_m")) limits["range_m"] = GetParams().Num("range_m", 0.0);
				if (GetParams().Has("bitrate_bps")) limits["bitrate_bps"] = GetParams().Num("bitrate_bps", 0.0);
				Declare(Capability::Messages("broadcast", limits));
				const auto type = PortTypeOf(GetParams());
				Params link;
				link.Add("channel", NumValue(static_cast<double>(GetParams().Int("channel", 1))));
				if (GetParams().Has("range_m")) link.Add("range_m", NumValue(GetParams().Num("range_m", 0.0)));
				DeclarePort("tx", type, Required(GetParams(), "emitter"), true, link);
				Params rx = link;
				rx.Merge(RateParams());
				DeclarePort("rx", type, Required(GetParams(), "receiver"), true, rx);
			}

			bool Radio::Send(const std::string& message, const std::string&) {
				// Broadcast link: the recipient is in the message itself (spec 06 §2).
				Port::Port* port = GetPort("tx");
				return port && port->Send(message);
			}

			std::vector<std::string> Radio::Receive() {
				std::vector<std::string> out;
				Port::Port* port = GetPort("rx");
				if (!port) return out;
				while (auto m = port->Receive()) out.push_back(std::move(*m));
				return out;
			}

			// --- Quadrotor -------------------------------------------------------------------

			void Quadrotor::Expand(const DeviceRegistry& registry) {
				const Params& p = GetParams();
				const std::string platform = Platform();
				static const std::pair<const char*, const char*> kMotors[] = {
					{"m_fl", "motor_fl"}, {"m_fr", "motor_fr"}, {"m_rl", "motor_rl"}, {"m_rr", "motor_rr"}};
				std::array<RotorMotor*, 4> motors{};
				for (std::size_t k = 0; k < 4; ++k) {
					Params mp;
					mp.Add("device", StrValue(Required(p, kMotors[k].second)));
					Pass(p, mp, {"port", "max_speed"});
					auto& node = AddVirtual(registry, "motor." + platform, kMotors[k].first, mp);
					motors[k] = dynamic_cast<RotorMotor*>(&node);
					if (!motors[k]) throw BuildError("motor." + platform + " is not a rotor motor");
				}

				Params fp;
				Pass(p, fp, {"max_climb", "max_speed_xy"});
				auto* fcu = dynamic_cast<FlightControlUnit*>(&AddVirtual(registry, "fcu.default", "fcu", fp));
				if (!fcu) throw BuildError("fcu.default is not a flight control unit");
				fcu->SetMotors(motors);

				Params ip;
				Pass(p, ip, {"inertial", "gyro", "accelerometer", "rate_hz", "port"});
				AddVirtual(registry, "imu." + platform, "imu", ip);
			}

			void RegisterUavDevices(DeviceRegistry& r) {
				r.Register<HeadNode>("head.default");
				r.Register<FlightControlUnit>("fcu.default");
				r.Register<Quadrotor>("quadrotor.webots", {{"motor_fl", StrValue("front left propeller")},
				                                           {"motor_fr", StrValue("front right propeller")},
				                                           {"motor_rl", StrValue("rear left propeller")},
				                                           {"motor_rr", StrValue("rear right propeller")},
				                                           {"inertial", StrValue("inertial unit")},
				                                           {"gyro", StrValue("gyro")}});
				r.Register<RotorMotor>("motor.webots");
				r.Register<Imu>("imu.webots", {{"inertial", StrValue("inertial unit")}, {"gyro", StrValue("gyro")}});
				r.Register<Gnss>("gnss.webots", {{"device", StrValue("gps")}});
				r.Register<Compass>("compass.webots", {{"device", StrValue("compass")}});
				r.Register<Barometer>("baro.webots", {{"device", StrValue("altimeter")}});
				r.Register<LedArray>("led.webots", {{"device", StrValue("front left led")}, {"device", StrValue("front right led")}});
				r.Register<Battery>("battery.webots", {{"device", StrValue("battery")}, {"voltage", NumValue(11.55)}});
				r.Register<Radio>("radio.webots",
				                  {{"emitter", StrValue("emitter")}, {"receiver", StrValue("receiver")}, {"channel", NumValue(1)}});
			}
		}
	}
}
