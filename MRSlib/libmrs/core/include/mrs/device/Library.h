#pragma once
// The device library: parts and identity maps from a search path of folders (spec 16 §6.3).
#include <filesystem>
#include <map>
#include <string>
#include <string_view>
#include <vector>

#include "mrs/device/Identity.h"
#include "mrs/protocol/Record.h"

namespace MRS {
	namespace Device {
		class DeviceLibrary {
		public:
			// Loads `parts/*.mrsd` and `*.mrsi` of each folder. A part name found in two folders is taken from
			// the first. A folder that does not exist is skipped. Throws BuildError for a file that does not parse.
			static DeviceLibrary FromSearchPath(const std::vector<std::filesystem::path>& folders);
			// The search path in the MRS_LIBRARY environment variable (';' separated on Windows, ':' elsewhere).
			static DeviceLibrary FromEnvironment();

			// Adds a part from .mrsd text unless one of that name exists. Throws BuildError when the text is not one
			// top-level D_X record (spec 16 §6.2).
			void AddPart(const std::string& name, std::string_view text);
			void AddMap(IdentityMap map) { maps_.push_back(std::move(map)); }

			// The part's D_X record, or null.
			const Protocol::Record* Part(const std::string& name) const;
			std::vector<const IdentityMap*> Maps() const;
			bool Empty() const { return parts_.empty() && maps_.empty(); }

		private:
			std::map<std::string, Protocol::Record> parts_;
			std::vector<IdentityMap> maps_;
		};

		// The prefix of a type key that includes a part: part.<name> (spec 16 §6.2).
		inline constexpr const char* kPartPrefix = "part.";
	}
}
