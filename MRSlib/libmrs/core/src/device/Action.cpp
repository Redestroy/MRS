#include "mrs/device/Action.h"

#include <cmath>
#include <cstring>
#include <limits>

namespace MRS {
	namespace Device {
		namespace {
			std::uint32_t FloatBits(float f) {
				std::uint32_t u;
				std::memcpy(&u, &f, sizeof u);
				return u;
			}
			float BitsFloat(std::uint32_t u) {
				float f;
				std::memcpy(&f, &u, sizeof f);
				return f;
			}
			std::uint64_t DoubleBits(double d) {
				std::uint64_t u;
				std::memcpy(&u, &d, sizeof u);
				return u;
			}
			double BitsDouble(std::uint64_t u) {
				double d;
				std::memcpy(&d, &u, sizeof d);
				return d;
			}
			std::uint64_t Join(std::uint32_t high, std::uint32_t low) {
				return (static_cast<std::uint64_t>(high) << 32) | low;
			}
			std::uint32_t High(std::uint64_t arg) { return static_cast<std::uint32_t>(arg >> 32); }
			std::uint32_t Low(std::uint64_t arg) { return static_cast<std::uint32_t>(arg & 0xFFFFFFFFu); }

			// Rounds to nearest binary32; a finite value that overflows is an error.
			std::optional<float> ToFloat(double v) {
				if (!std::isfinite(v)) return std::nullopt;
				float f = static_cast<float>(v);
				if (!std::isfinite(f)) return std::nullopt;
				return f;
			}
		}

		const std::vector<ActionInfo>& ActionRegistry() {
			static const std::vector<ActionInfo> registry = {
				{"A_N", "NULL", ArgLayout::NONE},
				{"A_W", "WAIT", ArgLayout::F64},
				{"A_I", "IMPOSSIBLE", ArgLayout::NONE},
				{"A_SY", "SYNC", ArgLayout::F64},
				{"A_L", "LED", ArgLayout::U32X2},
				{"A_TO", "TAKEOFF", ArgLayout::F32X2},
				{"A_LD", "LAND", ArgLayout::F32X2},
				{"A_HD", "HOLD", ArgLayout::F32X2},
				{"A_PXY", "POS_XY", ArgLayout::F32X2},
				{"A_PZY", "POS_Z_YAW", ArgLayout::F32X2},
				{"A_VXY", "VEL_XY", ArgLayout::F32X2},
				{"A_VZY", "VEL_Z_YAWRATE", ArgLayout::F32X2},
				{"A_MOT", "MOTOR_SPEED", ArgLayout::F64},
				{"A_CAM", "CAPTURE", ArgLayout::U32X2},
				{"A_GMB", "GIMBAL", ArgLayout::F32X2},
				{"A_REL", "RELEASE", ArgLayout::I64},
			};
			return registry;
		}

		std::optional<ActionInfo> FindAction(std::string_view code) {
			for (const auto& info : ActionRegistry())
				if (code == info.code) return info;
			return std::nullopt;
		}

		int ValueCount(ArgLayout layout) {
			switch (layout) {
			case ArgLayout::NONE: return 0;
			case ArgLayout::I64:
			case ArgLayout::F64: return 1;
			default: return 2;
			}
		}

		bool IsIntegerLayout(ArgLayout layout) {
			return layout == ArgLayout::I64 || layout == ArgLayout::I32X2 || layout == ArgLayout::U32X2;
		}

		double Action::FirstHalfAsDouble() const { return BitsFloat(High(arg)); }
		double Action::SecondHalfAsDouble() const { return BitsFloat(Low(arg)); }

		std::optional<std::uint64_t> PackArgument(ArgLayout layout, const std::vector<double>& values) {
			if (IsIntegerLayout(layout) || static_cast<int>(values.size()) != ValueCount(layout)) return std::nullopt;
			switch (layout) {
			case ArgLayout::NONE: return 0;
			case ArgLayout::F64:
				if (!std::isfinite(values[0])) return std::nullopt;
				return DoubleBits(values[0]);
			case ArgLayout::F32X2: {
				auto a = ToFloat(values[0]);
				auto b = ToFloat(values[1]);
				if (!a || !b) return std::nullopt;
				return Join(FloatBits(*a), FloatBits(*b));
			}
			default: return std::nullopt;
			}
		}

		std::optional<std::uint64_t> PackArgument(ArgLayout layout, const std::vector<std::int64_t>& values) {
			if ((layout != ArgLayout::NONE && !IsIntegerLayout(layout)) || static_cast<int>(values.size()) != ValueCount(layout))
				return std::nullopt;
			switch (layout) {
			case ArgLayout::NONE: return 0;
			case ArgLayout::I64: return static_cast<std::uint64_t>(values[0]);
			case ArgLayout::I32X2:
				for (auto v : values)
					if (v < std::numeric_limits<std::int32_t>::min() || v > std::numeric_limits<std::int32_t>::max()) return std::nullopt;
				return Join(static_cast<std::uint32_t>(static_cast<std::int32_t>(values[0])),
				            static_cast<std::uint32_t>(static_cast<std::int32_t>(values[1])));
			case ArgLayout::U32X2:
				for (auto v : values)
					if (v < 0 || v > static_cast<std::int64_t>(std::numeric_limits<std::uint32_t>::max())) return std::nullopt;
				return Join(static_cast<std::uint32_t>(values[0]), static_cast<std::uint32_t>(values[1]));
			default: return std::nullopt;
			}
		}

		std::vector<double> UnpackReals(ArgLayout layout, std::uint64_t arg) {
			switch (layout) {
			case ArgLayout::F64: return {BitsDouble(arg)};
			case ArgLayout::F32X2: return {BitsFloat(High(arg)), BitsFloat(Low(arg))};
			default: return {};
			}
		}

		std::vector<std::int64_t> UnpackIntegers(ArgLayout layout, std::uint64_t arg) {
			switch (layout) {
			case ArgLayout::I64: return {static_cast<std::int64_t>(arg)};
			case ArgLayout::I32X2:
				return {static_cast<std::int32_t>(High(arg)), static_cast<std::int32_t>(Low(arg))};
			case ArgLayout::U32X2: return {High(arg), Low(arg)};
			default: return {};
			}
		}
	}
}
