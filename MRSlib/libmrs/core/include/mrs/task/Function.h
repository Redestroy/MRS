#pragma once
// Functions for parametric actions (spec 02 §7) and the function registry (spec 02 §8).
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "mrs/world/Worldview.h"

namespace MRS {
	namespace Task {
		using RegistryFunction =
		    std::function<double(const Environment::Worldview& w, double t, const std::vector<double>& k)>;

		class FunctionRegistry {
		public:
			void Register(std::string key, RegistryFunction f, std::size_t n_coefficients, std::vector<std::string> fields_read);
			bool Has(const std::string& key) const;
			std::size_t CoefficientCount(const std::string& key) const;
			const std::vector<std::string>& FieldsRead(const std::string& key) const;

			// Calls the function when every field it reads is present and fresh; nullopt
			// otherwise, which fails the task with MISSING_FIELD (spec 02 §6).
			std::optional<double> Call(const std::string& key, const Environment::Worldview& w, double t,
			                           const std::vector<double>& k) const;

		private:
			struct Entry {
				RegistryFunction f;
				std::size_t n;
				std::vector<std::string> fields;
			};
			std::map<std::string, Entry> entries_;
		};

		// Registers the UAV functions of spec 02 §8: dist_to_target, dist_to_target_xy,
		// yaw_to_target, vel_to_target_x/_y/_z, layered_x/_y/_z.
		void RegisterUavFunctions(FunctionRegistry& registry);

		class Function {
		public:
			explicit Function(std::string code) : code_(std::move(code)) {}
			virtual ~Function() = default;
			const std::string& Code() const { return code_; }
			// nullopt when a field it reads is missing or stale.
			virtual std::optional<double> Evaluate(const Environment::Worldview& w, double t) const = 0;
			virtual void Fields(std::vector<std::string>& out) const { (void)out; }
			virtual std::unique_ptr<Function> Clone() const = 0;

		private:
			std::string code_;
		};

		using FunctionPtr = std::unique_ptr<Function>;

		class ConstantFunction : public Function {  // F_K
		public:
			explicit ConstantFunction(double value) : Function("F_K"), value_(value) {}
			std::optional<double> Evaluate(const Environment::Worldview&, double) const override { return value_; }
			FunctionPtr Clone() const override { return std::make_unique<ConstantFunction>(*this); }

		private:
			double value_;
		};

		class LinearFunction : public Function {  // F_L: c + sum(k_i * w[x_i])
		public:
			LinearFunction(std::vector<double> k, double c, std::vector<std::string> fields);
			std::optional<double> Evaluate(const Environment::Worldview& w, double t) const override;
			void Fields(std::vector<std::string>& out) const override;
			FunctionPtr Clone() const override { return std::make_unique<LinearFunction>(*this); }

		private:
			std::vector<double> k_;
			double c_;
			std::vector<std::string> fields_;
		};

		class RegistryCall : public Function {  // F_X
		public:
			RegistryCall(std::string key, std::vector<double> k, const FunctionRegistry& registry);
			std::optional<double> Evaluate(const Environment::Worldview& w, double t) const override;
			void Fields(std::vector<std::string>& out) const override;
			FunctionPtr Clone() const override { return std::make_unique<RegistryCall>(*this); }

		private:
			std::string key_;
			std::vector<double> k_;
			const FunctionRegistry* registry_;
		};

		class CombinedFunction : public Function {  // F_S (sum) and F_P (product)
		public:
			CombinedFunction(std::string code, std::vector<FunctionPtr> children);
			std::optional<double> Evaluate(const Environment::Worldview& w, double t) const override;
			void Fields(std::vector<std::string>& out) const override;
			FunctionPtr Clone() const override;

		private:
			std::vector<FunctionPtr> children_;
		};

		class ClampFunction : public Function {  // F_C
		public:
			ClampFunction(double lo, double hi, FunctionPtr child);
			std::optional<double> Evaluate(const Environment::Worldview& w, double t) const override;
			void Fields(std::vector<std::string>& out) const override { child_->Fields(out); }
			FunctionPtr Clone() const override { return std::make_unique<ClampFunction>(lo_, hi_, child_->Clone()); }

		private:
			double lo_, hi_;
			FunctionPtr child_;
		};
	}
}
