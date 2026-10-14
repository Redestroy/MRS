#pragma once
// Device identities, identity patterns and identity maps (spec 16 §2-§3).
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "mrs/device/Params.h"
#include "mrs/protocol/Record.h"

namespace MRS {
	namespace Device {
		class DeviceRegistry;

		// vendor:model[:rev][@serial] (spec 16 §2.1).
		struct Identity {
			std::string vendor, model, rev, serial;  // rev and serial may be empty

			// Nothing for a string that is not in the identity format.
			static std::optional<Identity> Parse(std::string_view text);
			std::string ToString() const;
		};

		// An identity whose rev may be `*` or left out and whose serial may be left out.
		struct IdentityPattern {
			std::string vendor, model, rev, serial;  // empty rev or serial: not constrained

			static std::optional<IdentityPattern> Parse(std::string_view text);
			bool Matches(const Identity& id) const;
			int Specificity() const { return (rev.empty() ? 0 : 1) + (serial.empty() ? 0 : 1); }
			std::string ToString() const;
		};

		// Matching of a requirement's identity against a scanned one (spec 04 §6.3, spec 16 §2.1). Both in the
		// identity format: pattern matching. Otherwise (version 0.1 free-form identities): exact equality.
		bool IdentityMatches(const std::string& pattern, const std::string& identity);

		// The type key `ignore` of an I_M: the device is known and deliberately not used.
		inline constexpr const char* kIgnoreType = "ignore";

		// One I_M record.
		struct IdentityMapping {
			IdentityPattern pattern;
			std::string type;  // registry key, part.<name>, or ignore
			Params params;
		};

		// The I_M records of a .mrsi file (spec 16 §3).
		class IdentityMap {
		public:
			// Throws BuildError on a record that is not I_M.
			static IdentityMap FromDocument(const Protocol::Document& doc);
			// Parses .mrsi text. Throws BuildError on a parse error.
			static IdentityMap Parse(std::string_view text);

			void Add(IdentityMapping mapping) { entries_.push_back(std::move(mapping)); }
			const std::vector<IdentityMapping>& Entries() const { return entries_; }
			// The most specific mapping that matches, or null. Throws BuildError when two equally specific
			// mappings with another type or other parameters match.
			const IdentityMapping* Find(const Identity& id) const;
			Protocol::Document ToDocument() const;

		private:
			std::vector<IdentityMapping> entries_;
		};

		// What an identity resolved to (spec 16 §3.2).
		struct IdentityMatch {
			enum class Source { RobotMap, Descriptor, Library, Registry };
			Source source = Source::Registry;
			std::string type;   // registry key, part.<name>, or ignore
			Params params;      // the I_M or descriptor parameters, without registry defaults
			std::optional<Protocol::Record> descriptor;  // the descriptor's D record, when it won
			bool Ignored() const { return type == kIgnoreType; }
		};

		const char* IdentitySourceName(IdentityMatch::Source source);

		// Parses a descriptor (spec 16 §2.2): one D record other than D_H, with its sub-records.
		// Throws BuildError otherwise.
		Protocol::Record ParseDescriptor(std::string_view text);

		// Resolves identities against the robot's map, the device's descriptor, the library maps and the
		// registry, in that order (spec 16 §3.2).
		class IdentityResolver {
		public:
			explicit IdentityResolver(const DeviceRegistry& registry, const IdentityMap* robot_map = nullptr,
			                          std::vector<const IdentityMap*> library = {})
			    : registry_(registry), robot_(robot_map), library_(std::move(library)) {}

			// Nothing when no source matches or the identity is not in the identity format.
			// Throws BuildError for an ambiguous source or a bad descriptor.
			std::optional<IdentityMatch> Resolve(const std::string& identity, const std::string& descriptor = {}) const;

		private:
			const DeviceRegistry& registry_;
			const IdentityMap* robot_;
			std::vector<const IdentityMap*> library_;
		};
	}
}
