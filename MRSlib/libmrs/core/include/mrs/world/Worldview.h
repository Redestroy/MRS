#pragma once
// Minimal worldview for WP1 (spec 05 §1-§4): named, typed, time-stamped fields with
// freshness. WP3 adds time series, the view router and processors on top of this.
#include <limits>
#include <map>
#include <optional>
#include <string>
#include <variant>
#include <vector>

namespace MRS {
	namespace Environment {
		using FieldValue = std::variant<double, bool, std::string>;  // scalar, bool, id

		struct WorldFieldEntry {
			FieldValue value;
			double stamp = 0.0;  // mission time of the information
			bool valid = true;
		};

		class Worldview {
		public:
			static constexpr double kNoMaxAge = std::numeric_limits<double>::infinity();

			Worldview();

			// Writers. A vector field is stored as its components (pose.enu.x, .y, .z).
			void SetScalar(const std::string& path, double value, double stamp);
			void SetBool(const std::string& path, bool value, double stamp);
			void SetId(const std::string& path, const std::string& value, double stamp);
			void SetVec3(const std::string& path, double x, double y, double z, double stamp);
			void Invalidate(const std::string& path, double stamp);  // valid = false (spec 05 §3)
			void Erase(const std::string& path);                     // the field becomes missing

			// Readers. Each returns nullopt when the field is missing, invalid, stale at
			// mission time t, or of another type.
			std::optional<double> Scalar(const std::string& path, double t) const;
			std::optional<bool> Bool(const std::string& path, double t) const;
			std::optional<std::string> Id(const std::string& path, double t) const;

			// A scalar or a group: TRUE when the field, or every component under it, is
			// present and at most max_age old (the field's default max_age when max_age < 0).
			bool IsFresh(const std::string& path, double t, double max_age = -1.0) const;
			bool Has(const std::string& path) const;

			// Raw access, without freshness, for binding and restoring target fields.
			std::optional<WorldFieldEntry> Raw(const std::string& path) const;
			void Restore(const std::string& path, const std::optional<WorldFieldEntry>& entry);

			// Paths that start with prefix (for example "det.person.").
			std::vector<std::string> PathsWithPrefix(const std::string& prefix) const;

			// Default max_age per field (spec 05 §4.1). A component inherits the max_age of
			// its vector field; fields not in the table never go stale.
			void SetMaxAge(const std::string& path, double max_age);
			double MaxAge(const std::string& path) const;

		private:
			const WorldFieldEntry* Fresh(const std::string& path, double t) const;

			std::map<std::string, WorldFieldEntry> fields_;
			std::map<std::string, double> max_age_;
		};
	}
}
