#pragma once
// Field requirements with a maximum age (plan §6.7): static and dynamic checks.
#include <set>
#include <string>
#include <vector>

#include "mrs/world/Worldview.h"

namespace MRS {
	namespace Environment {
		struct FieldRequirement {
			std::string path;
			double max_age = -1.0;  // < 0: the field's default max_age
		};

		class WorldviewRequirements {
		public:
			WorldviewRequirements() = default;
			WorldviewRequirements(std::initializer_list<FieldRequirement> r) : fields_(r) {}
			void Add(std::string path, double max_age = -1.0) { fields_.push_back({std::move(path), max_age}); }
			const std::vector<FieldRequirement>& Fields() const { return fields_; }

			// Dynamic: every field is fresh now.
			bool Satisfies(const Worldview& w, double t) const { return Missing(w, t).empty(); }
			std::vector<std::string> Missing(const Worldview& w, double t) const;

			// Static: the robot can provide every field (a field or one of its parents is in `provided`,
			// for example the self model's field list).
			bool Satisfiable(const std::set<std::string>& provided) const;

		private:
			std::vector<FieldRequirement> fields_;
		};
	}
}
