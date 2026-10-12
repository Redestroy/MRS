#include "mrs/device/Library.h"

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <sstream>

#include "mrs/BuildError.h"
#include "mrs/protocol/Parser.h"

namespace MRS {
	namespace Device {
		namespace {
			std::string ReadText(const std::filesystem::path& path) {
				std::ifstream in(path, std::ios::binary);
				std::stringstream buffer;
				buffer << in.rdbuf();
				return buffer.str();
			}

			std::vector<std::filesystem::path> Sorted(const std::filesystem::path& dir, const std::string& extension) {
				std::vector<std::filesystem::path> out;
				std::error_code ec;
				if (!std::filesystem::is_directory(dir, ec)) return out;
				for (const auto& entry : std::filesystem::directory_iterator(dir, ec))
					if (entry.is_regular_file() && entry.path().extension() == extension) out.push_back(entry.path());
				std::sort(out.begin(), out.end());
				return out;
			}
		}

		DeviceLibrary DeviceLibrary::FromSearchPath(const std::vector<std::filesystem::path>& folders) {
			DeviceLibrary lib;
			for (const auto& folder : folders) {
				for (const auto& path : Sorted(folder / "parts", ".mrsd")) {
					try {
						lib.AddPart(path.stem().string(), ReadText(path));
					} catch (const BuildError& e) {
						throw BuildError(path.string() + ": " + e.what());
					}
				}
				for (const auto& path : Sorted(folder, ".mrsi")) {
					try {
						lib.AddMap(IdentityMap::Parse(ReadText(path)));
					} catch (const BuildError& e) {
						throw BuildError(path.string() + ": " + e.what());
					}
				}
			}
			return lib;
		}

		DeviceLibrary DeviceLibrary::FromEnvironment() {
			std::vector<std::filesystem::path> folders;
			const char* env = std::getenv("MRS_LIBRARY");
			if (!env) return {};
#ifdef _WIN32
			const char separator = ';';
#else
			const char separator = ':';
#endif
			std::string value(env);
			std::size_t start = 0;
			while (start <= value.size()) {
				const auto end = value.find(separator, start);
				const std::string item = value.substr(start, end == std::string::npos ? std::string::npos : end - start);
				if (!item.empty()) folders.emplace_back(item);
				if (end == std::string::npos) break;
				start = end + 1;
			}
			return FromSearchPath(folders);
		}

		void DeviceLibrary::AddPart(const std::string& name, std::string_view text) {
			auto result = Protocol::Parse(text);
			if (!result.Ok())
				throw BuildError("part " + name + ": " + std::string(Protocol::ErrorClassName(result.error->error_class)) + " at byte " +
				                 std::to_string(result.error->offset) + ": " + result.error->message);
			const Protocol::Record* root = nullptr;
			for (const auto& rec : result.document.records) {
				if (rec.kind == '@') continue;
				if (rec.code != "D_X") throw BuildError("part " + name + ": a part is one D_X record, found " + rec.code);
				if (root) throw BuildError("part " + name + ": a part holds one top-level D_X record");
				root = &rec;
			}
			if (!root) throw BuildError("part " + name + " has no D_X record");
			parts_.emplace(name, *root);
		}

		const Protocol::Record* DeviceLibrary::Part(const std::string& name) const {
			auto it = parts_.find(name);
			return it == parts_.end() ? nullptr : &it->second;
		}

		std::vector<const IdentityMap*> DeviceLibrary::Maps() const {
			std::vector<const IdentityMap*> out;
			for (const auto& m : maps_) out.push_back(&m);
			return out;
		}
	}
}
