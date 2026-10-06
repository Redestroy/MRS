#pragma once
// The error thrown when a robot cannot be built from its definition (spec 04).
#include <stdexcept>
#include <string>

namespace MRS {
	class BuildError : public std::runtime_error {
	public:
		explicit BuildError(const std::string& message) : std::runtime_error(message) {}
	};
}
