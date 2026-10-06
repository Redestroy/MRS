#pragma once
// Actions and their 64-bit argument layouts (spec 02 §1, §2 and §4).
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace MRS {
	namespace Device {
		enum class ArgLayout { NONE, I64, F64, F32X2, I32X2, U32X2 };

		struct ActionInfo {
			const char* code;   // "A_PXY"
			const char* name;   // "POS_XY"
			ArgLayout layout;
		};

		// The leaf action codes of the version 0.1 registry (spec 02 §4.1-4.3).
		// A_MAP and A_FN are structural and have no layout.
		const std::vector<ActionInfo>& ActionRegistry();
		std::optional<ActionInfo> FindAction(std::string_view code);

		// Number of text values of a layout: 0, 1 or 2.
		int ValueCount(ArgLayout layout);
		bool IsIntegerLayout(ArgLayout layout);

		struct Action {
			std::string code;
			std::uint64_t arg = 0;

			// Value 1 is always the high half (spec 02 §2).
			double FirstHalfAsDouble() const;
			double SecondHalfAsDouble() const;
		};

		// A combined action: target -> action, all dispatched in the same tick (spec 02 §5).
		// A single action is a map with one entry whose target is "any".
		struct ActionMapEntry {
			std::string target;  // a device node name, or "any" (route by action code)
			Action action;
		};

		struct ActionMap {
			std::vector<ActionMapEntry> entries;
			bool combined = false;  // written as A_MAP
		};

		// What an actuator reports for a dispatched action (spec 03 §8.2).
		enum class ActionStatus { DONE, RUNNING, REJECTED, FAILED };

		// Packs text values by the layout rules. Returns nullopt if the layout takes the
		// other value type, the count is wrong, or a value is outside the layout's range
		// (binary32 range for F32X2, int32 for I32X2, uint32 for U32X2).
		std::optional<std::uint64_t> PackArgument(ArgLayout layout, const std::vector<double>& values);       // NONE, F64, F32X2
		std::optional<std::uint64_t> PackArgument(ArgLayout layout, const std::vector<std::int64_t>& values); // NONE, I64, I32X2, U32X2

		// The inverses of PackArgument.
		std::vector<double> UnpackReals(ArgLayout layout, std::uint64_t arg);
		std::vector<std::int64_t> UnpackIntegers(ArgLayout layout, std::uint64_t arg);
	}
}
