#pragma once
// The condition family (spec 03 §4). Conditions are three-valued: a field that is
// missing or stale makes them UNKNOWN (spec 05 §3).
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "mrs/world/GeoReference.h"
#include "mrs/world/Worldview.h"

namespace MRS {
	namespace Task {
		enum class Truth { False, True, Unknown };

		// Kleene helpers.
		Truth Not(Truth a);
		Truth FromBool(bool b);

		// Values of the C_L op slot, unchanged from the 2021 core (spec 03 §4.1).
		enum class LogicalOperation { AND = 1, OR = 2, NOT = 3, XOR = 4, NAND = 5, NOR = 6, NXOR = 7 };

		struct EvalContext {
			const Environment::Worldview& w;
			double t = 0.0;              // mission time, s
			double context_start = 0.0;  // C_W context start of the task that holds the condition (spec 03 §8.3)
			const Environment::GeoReference* geo = nullptr;  // for C_G
			// Pool state of a task id (spec 03 §3.2), for C_S. Unset or nullopt: UNKNOWN.
			std::function<std::optional<std::string>(const std::string& task_id)> pool_state;
		};

		class Condition {
		public:
			explicit Condition(std::string code) : code_(std::move(code)) {}
			virtual ~Condition() = default;

			const std::string& Code() const { return code_; }
			virtual Truth Evaluate(const EvalContext& c) const = 0;
			virtual std::unique_ptr<Condition> Clone() const = 0;

			// Worldview fields the condition reads (spec 03 §5, implicit requirements).
			virtual void Fields(std::vector<std::string>& out) const { (void)out; }
			// The behaviour-library qualifier: the predicate name for C_?, otherwise "any".
			virtual std::string Qualifier() const { return "any"; }
			// For C_?: the wanted value. Other conditions want TRUE.
			virtual bool WantedValue() const { return true; }
			// Writes the target.* fields of spec 03 §7 and returns the paths written.
			// C_G needs the geo reference; the others ignore it.
			virtual std::vector<std::string> Bind(Environment::Worldview& w, double t, const Environment::GeoReference* geo) const {
				(void)w;
				(void)t;
				(void)geo;
				return {};
			}

		private:
			std::string code_;
		};

		using ConditionPtr = std::unique_ptr<Condition>;

		class NullCondition : public Condition {  // C_N
		public:
			NullCondition() : Condition("C_N") {}
			Truth Evaluate(const EvalContext&) const override { return Truth::True; }
			ConditionPtr Clone() const override { return std::make_unique<NullCondition>(); }
		};

		class FalseCondition : public Condition {  // C_F
		public:
			FalseCondition() : Condition("C_F") {}
			Truth Evaluate(const EvalContext&) const override { return Truth::False; }
			ConditionPtr Clone() const override { return std::make_unique<FalseCondition>(); }
		};

		class PredicateCondition : public Condition {  // C_?
		public:
			PredicateCondition(std::string predicate, bool value);
			Truth Evaluate(const EvalContext& c) const override;
			ConditionPtr Clone() const override { return std::make_unique<PredicateCondition>(*this); }
			void Fields(std::vector<std::string>& out) const override { out.push_back(predicate_); }
			std::string Qualifier() const override { return predicate_; }
			bool WantedValue() const override { return value_; }
			std::vector<std::string> Bind(Environment::Worldview& w, double t, const Environment::GeoReference* geo) const override;
			const std::string& Predicate() const { return predicate_; }

		private:
			std::string predicate_;
			bool value_;
		};

		class ParameterCondition : public Condition {  // C_m
		public:
			ParameterCondition(std::string field, std::string op, double value, double tol);
			Truth Evaluate(const EvalContext& c) const override;
			ConditionPtr Clone() const override { return std::make_unique<ParameterCondition>(*this); }
			void Fields(std::vector<std::string>& out) const override { out.push_back(field_); }

		private:
			std::string field_, op_;
			double value_, tol_;
		};

		class AbsoluteTimeCondition : public Condition {  // C_T, milliseconds of mission time
		public:
			explicit AbsoluteTimeCondition(long long t_ms) : Condition("C_T"), t_ms_(t_ms) {}
			Truth Evaluate(const EvalContext& c) const override;
			ConditionPtr Clone() const override { return std::make_unique<AbsoluteTimeCondition>(*this); }
			void Fields(std::vector<std::string>& out) const override { out.push_back("time"); }

		private:
			long long t_ms_;
		};

		class ElapsedTimeCondition : public Condition {  // C_W, seconds since the context start
		public:
			explicit ElapsedTimeCondition(double d) : Condition("C_W"), d_(d) {}
			Truth Evaluate(const EvalContext& c) const override;
			ConditionPtr Clone() const override { return std::make_unique<ElapsedTimeCondition>(*this); }
			void Fields(std::vector<std::string>& out) const override { out.push_back("time"); }

		private:
			double d_;
		};

		class LogicCondition : public Condition {  // C_L
		public:
			LogicCondition(LogicalOperation op, std::vector<ConditionPtr> children);
			Truth Evaluate(const EvalContext& c) const override;
			ConditionPtr Clone() const override;
			void Fields(std::vector<std::string>& out) const override;
			LogicalOperation Operation() const { return op_; }

		private:
			LogicalOperation op_;
			std::vector<ConditionPtr> children_;
		};

		class ViewMatchCondition : public Condition {  // C_V, V_PEER and V_DET only (spec 05 §6)
		public:
			static ViewMatchCondition Peer(long long robot);
			static ViewMatchCondition Detection(std::string object_class, double x, double y, double z);
			Truth Evaluate(const EvalContext& c) const override;
			ConditionPtr Clone() const override { return std::make_unique<ViewMatchCondition>(*this); }
			void Fields(std::vector<std::string>& out) const override;

		private:
			ViewMatchCondition() : Condition("C_V") {}
			std::string view_code_;
			long long robot_ = 0;
			std::string class_;
			double x_ = 0, y_ = 0, z_ = 0;
		};

		class TaskStateCondition : public Condition {  // C_S
		public:
			TaskStateCondition(std::string task_id, std::string state);
			Truth Evaluate(const EvalContext& c) const override;
			ConditionPtr Clone() const override { return std::make_unique<TaskStateCondition>(*this); }
			const std::string& TaskId() const { return task_id_; }

		private:
			std::string task_id_, state_;
		};

		class PositionCondition : public Condition {  // C_P
		public:
			PositionCondition(double x, double y, double yaw, double tol_xy, double tol_yaw);
			Truth Evaluate(const EvalContext& c) const override;
			ConditionPtr Clone() const override { return std::make_unique<PositionCondition>(*this); }
			void Fields(std::vector<std::string>& out) const override;
			std::vector<std::string> Bind(Environment::Worldview& w, double t, const Environment::GeoReference* geo) const override;

		private:
			double x_, y_, yaw_, tol_xy_, tol_yaw_;
		};

		class PositionCondition3D : public Condition {  // C_P3
		public:
			PositionCondition3D(double x, double y, double z, double tol_xy, double tol_z, double yaw, double tol_yaw);
			Truth Evaluate(const EvalContext& c) const override;
			ConditionPtr Clone() const override { return std::make_unique<PositionCondition3D>(*this); }
			void Fields(std::vector<std::string>& out) const override;
			std::vector<std::string> Bind(Environment::Worldview& w, double t, const Environment::GeoReference* geo) const override;

		private:
			double x_, y_, z_, tol_xy_, tol_z_, yaw_, tol_yaw_;
		};

		class GeoPositionCondition : public Condition {  // C_G
		public:
			GeoPositionCondition(double lat, double lon, double alt_amsl, double tol_xy, double tol_z);
			Truth Evaluate(const EvalContext& c) const override;
			ConditionPtr Clone() const override { return std::make_unique<GeoPositionCondition>(*this); }
			void Fields(std::vector<std::string>& out) const override;
			std::vector<std::string> Bind(Environment::Worldview& w, double t, const Environment::GeoReference* geo) const override;

		private:
			double lat_, lon_, alt_, tol_xy_, tol_z_;
		};

		class AltitudeCondition : public Condition {  // C_H
		public:
			AltitudeCondition(double h, double tol) : Condition("C_H"), h_(h), tol_(tol) {}
			Truth Evaluate(const EvalContext& c) const override;
			ConditionPtr Clone() const override { return std::make_unique<AltitudeCondition>(*this); }
			void Fields(std::vector<std::string>& out) const override { out.push_back("alt.agl"); }
			std::vector<std::string> Bind(Environment::Worldview& w, double t, const Environment::GeoReference* geo) const override;

		private:
			double h_, tol_;
		};

		// Wraps an angle difference to (-pi, pi] (spec 03 §4).
		double WrapAngle(double a);
	}
}
