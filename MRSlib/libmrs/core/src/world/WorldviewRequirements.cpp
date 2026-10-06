#include "mrs/world/WorldviewRequirements.h"

namespace MRS {
	namespace Environment {
		std::vector<std::string> WorldviewRequirements::Missing(const Worldview& w, double t) const {
			std::vector<std::string> out;
			for (const auto& f : fields_)
				if (!w.IsFresh(f.path, t, f.max_age)) out.push_back(f.path);
			return out;
		}

		bool WorldviewRequirements::Satisfiable(const std::set<std::string>& provided) const {
			for (const auto& f : fields_) {
				std::string p = f.path;
				bool found = false;
				while (!found) {
					if (provided.count(p)) found = true;
					const auto dot = p.rfind('.');
					if (dot == std::string::npos) break;
					p.resize(dot);
				}
				if (!found) return false;
			}
			return true;
		}
	}
}
