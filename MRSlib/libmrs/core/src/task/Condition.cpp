#include "mrs/task/Condition.h"

#include <cmath>

namespace MRS {
	namespace Task {
		namespace {
			constexpr double kPi = 3.14159265358979323846;

			struct Pose {
				double x, y, z;
			};

			std::optional<Pose> ReadPose(const EvalContext& c) {
				auto x = c.w.Scalar("pose.enu.x", c.t), y = c.w.Scalar("pose.enu.y", c.t), z = c.w.Scalar("pose.enu.z", c.t);
				if (!x || !y || !z) return std::nullopt;
				return Pose{*x, *y, *z};
			}

			// Yaw check: TRUE when tol < 0 (ignore yaw) or the yaw is within tol.
			Truth YawOk(const EvalContext& c, double yaw, double tol) {
				if (tol < 0) return Truth::True;
				auto current = c.w.Scalar("att.yaw", c.t);
				if (!current) return Truth::Unknown;
				return FromBool(std::fabs(WrapAngle(*current - yaw)) <= tol);
			}

			Truth And(Truth a, Truth b) {
				if (a == Truth::False || b == Truth::False) return Truth::False;
				if (a == Truth::Unknown || b == Truth::Unknown) return Truth::Unknown;
				return Truth::True;
			}

			void SetTarget(Environment::Worldview& w, double t, std::vector<std::string>& written, const char* name, double v) {
				const std::string path = std::string("target.") + name;
				w.SetScalar(path, v, t);
				written.push_back(path);
			}

			Truth Position3D(const EvalContext& c, double x, double y, double z, double tol_xy, double tol_z, double yaw, double tol_yaw) {
				auto p = ReadPose(c);
				Truth pos = Truth::Unknown;
				if (p) pos = FromBool(std::hypot(p->x - x, p->y - y) <= tol_xy && std::fabs(p->z - z) <= tol_z);
				return And(pos, YawOk(c, yaw, tol_yaw));
			}
		}

		Truth Not(Truth a) {
			if (a == Truth::Unknown) return a;
			return a == Truth::True ? Truth::False : Truth::True;
		}

		Truth FromBool(bool b) { return b ? Truth::True : Truth::False; }

		double WrapAngle(double a) {
			a = std::fmod(a, 2 * kPi);
			if (a <= -kPi) a += 2 * kPi;
			if (a > kPi) a -= 2 * kPi;
			return a;
		}

		// ---- C_?

		PredicateCondition::PredicateCondition(std::string predicate, bool value)
		    : Condition("C_?"), predicate_(std::move(predicate)), value_(value) {}

		Truth PredicateCondition::Evaluate(const EvalContext& c) const {
			auto v = c.w.Bool(predicate_, c.t);
			if (!v) return Truth::Unknown;
			return FromBool(*v == value_);
		}

		std::vector<std::string> PredicateCondition::Bind(Environment::Worldview& w, double t, const Environment::GeoReference*) const {
			w.SetId("target.predicate", predicate_, t);
			w.SetBool("target.value", value_, t);
			return {"target.predicate", "target.value"};
		}

		// ---- C_m

		ParameterCondition::ParameterCondition(std::string field, std::string op, double value, double tol)
		    : Condition("C_m"), field_(std::move(field)), op_(std::move(op)), value_(value), tol_(tol) {}

		Truth ParameterCondition::Evaluate(const EvalContext& c) const {
			auto v = c.w.Scalar(field_, c.t);
			if (!v) return Truth::Unknown;
			const double x = *v;
			if (op_ == "lt") return FromBool(x < value_);
			if (op_ == "le") return FromBool(x <= value_);
			if (op_ == "eq") return FromBool(std::fabs(x - value_) <= tol_);
			if (op_ == "ne") return FromBool(std::fabs(x - value_) > tol_);
			if (op_ == "ge") return FromBool(x >= value_);
			return FromBool(x > value_);  // gt
		}

		// ---- C_T and C_W

		Truth AbsoluteTimeCondition::Evaluate(const EvalContext& c) const {
			return FromBool(c.t * 1000.0 >= static_cast<double>(t_ms_));
		}

		Truth ElapsedTimeCondition::Evaluate(const EvalContext& c) const { return FromBool(c.t - c.context_start >= d_); }

		// ---- C_L

		LogicCondition::LogicCondition(LogicalOperation op, std::vector<ConditionPtr> children)
		    : Condition("C_L"), op_(op), children_(std::move(children)) {}

		Truth LogicCondition::Evaluate(const EvalContext& c) const {
			std::vector<Truth> v;
			for (const auto& child : children_) v.push_back(child->Evaluate(c));
			auto all = [&] {
				Truth r = Truth::True;
				for (Truth x : v) r = And(r, x);
				return r;
			};
			auto any = [&] {
				Truth r = Truth::False;
				for (Truth x : v) r = Not(And(Not(r), Not(x)));
				return r;
			};
			auto parity = [&] {
				int n = 0;
				for (Truth x : v) {
					if (x == Truth::Unknown) return Truth::Unknown;
					if (x == Truth::True) ++n;
				}
				return FromBool(n % 2 == 1);
			};
			switch (op_) {
			case LogicalOperation::AND: return all();
			case LogicalOperation::OR: return any();
			case LogicalOperation::NOT: return Not(v.at(0));
			case LogicalOperation::XOR: return parity();
			case LogicalOperation::NAND: return Not(all());
			case LogicalOperation::NOR: return Not(any());
			case LogicalOperation::NXOR: return Not(parity());
			}
			return Truth::Unknown;
		}

		ConditionPtr LogicCondition::Clone() const {
			std::vector<ConditionPtr> copy;
			for (const auto& child : children_) copy.push_back(child->Clone());
			return std::make_unique<LogicCondition>(op_, std::move(copy));
		}

		void LogicCondition::Fields(std::vector<std::string>& out) const {
			for (const auto& child : children_) child->Fields(out);
		}

		// ---- C_V

		ViewMatchCondition ViewMatchCondition::Peer(long long robot) {
			ViewMatchCondition c;
			c.view_code_ = "V_PEER";
			c.robot_ = robot;
			return c;
		}

		ViewMatchCondition ViewMatchCondition::Detection(std::string object_class, double x, double y, double z) {
			ViewMatchCondition c;
			c.view_code_ = "V_DET";
			c.class_ = std::move(object_class);
			c.x_ = x;
			c.y_ = y;
			c.z_ = z;
			return c;
		}

		Truth ViewMatchCondition::Evaluate(const EvalContext& c) const {
			if (view_code_ == "V_PEER") return FromBool(c.w.IsFresh("peer.r" + std::to_string(robot_) + ".pose.enu", c.t));
			// V_DET: a fresh detection of the class within 2 m (spec 05 §6).
			const std::string prefix = "det." + class_ + ".";
			for (const auto& path : c.w.PathsWithPrefix(prefix)) {
				const std::string suffix = ".enu.x";
				if (path.size() <= suffix.size() || path.compare(path.size() - suffix.size(), suffix.size(), suffix) != 0) continue;
				const std::string base = path.substr(0, path.size() - 2);  // det.<class>.<n>.enu
				auto x = c.w.Scalar(base + ".x", c.t), y = c.w.Scalar(base + ".y", c.t), z = c.w.Scalar(base + ".z", c.t);
				if (x && y && z && std::sqrt((*x - x_) * (*x - x_) + (*y - y_) * (*y - y_) + (*z - z_) * (*z - z_)) <= 2.0)
					return Truth::True;
			}
			return Truth::False;
		}

		void ViewMatchCondition::Fields(std::vector<std::string>& out) const {
			if (view_code_ == "V_PEER") out.push_back("peer.r" + std::to_string(robot_) + ".pose.enu");
		}

		// ---- C_S

		TaskStateCondition::TaskStateCondition(std::string task_id, std::string state)
		    : Condition("C_S"), task_id_(std::move(task_id)), state_(std::move(state)) {}

		Truth TaskStateCondition::Evaluate(const EvalContext& c) const {
			if (!c.pool_state) return Truth::Unknown;
			auto s = c.pool_state(task_id_);
			if (!s) return Truth::Unknown;
			return FromBool(*s == state_);
		}

		// ---- C_P

		PositionCondition::PositionCondition(double x, double y, double yaw, double tol_xy, double tol_yaw)
		    : Condition("C_P"), x_(x), y_(y), yaw_(yaw), tol_xy_(tol_xy), tol_yaw_(tol_yaw) {}

		Truth PositionCondition::Evaluate(const EvalContext& c) const {
			auto x = c.w.Scalar("pose.enu.x", c.t), y = c.w.Scalar("pose.enu.y", c.t);
			Truth pos = (x && y) ? FromBool(std::hypot(*x - x_, *y - y_) <= tol_xy_) : Truth::Unknown;
			return And(pos, YawOk(c, yaw_, tol_yaw_));
		}

		void PositionCondition::Fields(std::vector<std::string>& out) const {
			out.push_back("pose.enu");
			if (tol_yaw_ >= 0) out.push_back("att.yaw");
		}

		std::vector<std::string> PositionCondition::Bind(Environment::Worldview& w, double t, const Environment::GeoReference*) const {
			std::vector<std::string> written;
			SetTarget(w, t, written, "x", x_);
			SetTarget(w, t, written, "y", y_);
			SetTarget(w, t, written, "yaw", yaw_);
			SetTarget(w, t, written, "tol_xy", tol_xy_);
			SetTarget(w, t, written, "tol_yaw", tol_yaw_);
			if (auto z = w.Scalar("pose.enu.z", t)) SetTarget(w, t, written, "z", *z);
			return written;
		}

		// ---- C_P3

		PositionCondition3D::PositionCondition3D(double x, double y, double z, double tol_xy, double tol_z, double yaw, double tol_yaw)
		    : Condition("C_P3"), x_(x), y_(y), z_(z), tol_xy_(tol_xy), tol_z_(tol_z), yaw_(yaw), tol_yaw_(tol_yaw) {}

		Truth PositionCondition3D::Evaluate(const EvalContext& c) const {
			return Position3D(c, x_, y_, z_, tol_xy_, tol_z_, yaw_, tol_yaw_);
		}

		void PositionCondition3D::Fields(std::vector<std::string>& out) const {
			out.push_back("pose.enu");
			if (tol_yaw_ >= 0) out.push_back("att.yaw");
		}

		std::vector<std::string> PositionCondition3D::Bind(Environment::Worldview& w, double t, const Environment::GeoReference*) const {
			std::vector<std::string> written;
			SetTarget(w, t, written, "x", x_);
			SetTarget(w, t, written, "y", y_);
			SetTarget(w, t, written, "z", z_);
			SetTarget(w, t, written, "tol_xy", tol_xy_);
			SetTarget(w, t, written, "tol_z", tol_z_);
			SetTarget(w, t, written, "yaw", yaw_);
			SetTarget(w, t, written, "tol_yaw", tol_yaw_);
			return written;
		}

		// ---- C_G

		GeoPositionCondition::GeoPositionCondition(double lat, double lon, double alt_amsl, double tol_xy, double tol_z)
		    : Condition("C_G"), lat_(lat), lon_(lon), alt_(alt_amsl), tol_xy_(tol_xy), tol_z_(tol_z) {}

		Truth GeoPositionCondition::Evaluate(const EvalContext& c) const {
			if (!c.geo) return Truth::Unknown;
			const auto e = c.geo->ToEnu(lat_, lon_, alt_);
			return Position3D(c, e.x, e.y, e.z, tol_xy_, tol_z_, 0.0, -1.0);
		}

		void GeoPositionCondition::Fields(std::vector<std::string>& out) const { out.push_back("pose.enu"); }

		std::vector<std::string> GeoPositionCondition::Bind(Environment::Worldview& w, double t,
		                                                    const Environment::GeoReference* geo) const {
			if (!geo) return {};
			const auto e = geo->ToEnu(lat_, lon_, alt_);
			std::vector<std::string> written;
			SetTarget(w, t, written, "x", e.x);
			SetTarget(w, t, written, "y", e.y);
			SetTarget(w, t, written, "z", e.z);
			SetTarget(w, t, written, "tol_xy", tol_xy_);
			SetTarget(w, t, written, "tol_z", tol_z_);
			if (auto yaw = w.Scalar("att.yaw", t)) SetTarget(w, t, written, "yaw", *yaw);
			SetTarget(w, t, written, "tol_yaw", -1.0);
			return written;
		}

		// ---- C_H

		Truth AltitudeCondition::Evaluate(const EvalContext& c) const {
			auto agl = c.w.Scalar("alt.agl", c.t);
			if (!agl) return Truth::Unknown;
			return FromBool(std::fabs(*agl - h_) <= tol_);
		}

		std::vector<std::string> AltitudeCondition::Bind(Environment::Worldview& w, double t, const Environment::GeoReference*) const {
			std::vector<std::string> written;
			// target.z = h + ground height, where ground height = pose.enu.z - alt.agl.
			auto z = w.Scalar("pose.enu.z", t), agl = w.Scalar("alt.agl", t);
			SetTarget(w, t, written, "z", h_ + ((z && agl) ? *z - *agl : 0.0));
			SetTarget(w, t, written, "tol_z", tol_);
			return written;
		}
	}
}
