#pragma once
// Helpers for tests that read the spec examples (spec 07 §4).
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace MRS {
	namespace Test {
		inline std::filesystem::path ExamplesDir() { return std::filesystem::path(MRS_SPEC_EXAMPLES); }

		inline std::string ReadFile(const std::filesystem::path& path) {
			std::ifstream in(path, std::ios::binary);
			std::stringstream buffer;
			buffer << in.rdbuf();
			return buffer.str();
		}

		// Files directly in `dir` whose extension starts with ".mrs", sorted by name.
		inline std::vector<std::filesystem::path> ProtocolFiles(const std::filesystem::path& dir) {
			std::vector<std::filesystem::path> files;
			for (const auto& entry : std::filesystem::directory_iterator(dir))
				if (entry.is_regular_file() && entry.path().extension().string().rfind(".mrs", 0) == 0)
					files.push_back(entry.path());
			std::sort(files.begin(), files.end());
			return files;
		}
	}
}
