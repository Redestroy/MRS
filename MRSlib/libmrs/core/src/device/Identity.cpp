#include "mrs/device/Identity.h"

#include "mrs/BuildError.h"
#include "mrs/device/DeviceRegistry.h"
#include "mrs/protocol/Parser.h"

namespace MRS {
	namespace Device {
		using Protocol::Field;
		using Protocol::FieldType;
		using Protocol::Record;

		namespace {
			bool PartChar(char c) {
				return (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_' || c == '-' || c == '.';
			}
			bool ValidPart(std::string_view s) {
				if (s.empty()) return false;
				for (char c : s)
					if (!PartChar(c)) return false;
				return true;
			}
			bool ValidSerial(std::string_view s) {
				if (s.empty()) return false;
				for (char c : s)
					if (c == '"' || c == ' ' || c == '\t' || c == '\n' || c == '\r') return false;
				return true;
			}

			// Splits vendor:model[:rev][@serial]. `star` allows a rev of `*`.
			bool Split(std::string_view text, bool star, std::string& vendor, std::string& model, std::string& rev,
			           std::string& serial) {
				const auto at = text.find('@');
				std::string_view head = text.substr(0, at);
				if (at != std::string_view::npos) {
					serial = std::string(text.substr(at + 1));
					if (!ValidSerial(serial)) return false;
				}
				const auto c1 = head.find(':');
				if (c1 == std::string_view::npos) return false;
				const auto c2 = head.find(':', c1 + 1);
				vendor = std::string(head.substr(0, c1));
				model = std::string(head.substr(c1 + 1, c2 == std::string_view::npos ? std::string_view::npos : c2 - c1 - 1));
				if (c2 != std::string_view::npos) {
					rev = std::string(head.substr(c2 + 1));
					if (!(star && rev == "*") && !ValidPart(rev)) return false;
				}
				return ValidPart(vendor) && ValidPart(model);
			}

			std::string Join(const std::string& vendor, const std::string& model, const std::string& rev, const std::string& serial) {
				std::string s = vendor + ":" + model;
				if (!rev.empty()) s += ":" + rev;
				if (!serial.empty()) s += "@" + serial;
				return s;
			}
		}

		std::optional<Identity> Identity::Parse(std::string_view text) {
			Identity id;
			if (!Split(text, false, id.vendor, id.model, id.rev, id.serial)) return std::nullopt;
			return id;
		}

		std::string Identity::ToString() const { return Join(vendor, model, rev, serial); }

		std::optional<IdentityPattern> IdentityPattern::Parse(std::string_view text) {
			IdentityPattern p;
			if (!Split(text, true, p.vendor, p.model, p.rev, p.serial)) return std::nullopt;
			if (p.rev == "*") p.rev.clear();
			return p;
		}

		bool IdentityPattern::Matches(const Identity& id) const {
			return vendor == id.vendor && model == id.model && (rev.empty() || rev == id.rev) && (serial.empty() || serial == id.serial);
		}

		std::string IdentityPattern::ToString() const { return Join(vendor, model, rev.empty() && !serial.empty() ? "*" : rev, serial); }

		bool IdentityMatches(const std::string& pattern, const std::string& identity) {
			auto p = IdentityPattern::Parse(pattern);
			auto id = Identity::Parse(identity);
			if (p && id) return p->Matches(*id);
			return pattern == identity;
		}

		namespace {
			bool SameMapping(const IdentityMapping& a, const IdentityMapping& b) {
				if (a.type != b.type) return false;
				const auto fa = a.params.ToFields(), fb = b.params.ToFields();
				if (fa.size() != fb.size()) return false;
				for (std::size_t k = 0; k < fa.size(); ++k)
					if (fa[k] != fb[k]) return false;
				return true;
			}
		}

		IdentityMap IdentityMap::FromDocument(const Protocol::Document& doc) {
			IdentityMap map;
			for (const Record& rec : doc.records) {
				if (rec.kind == '@') continue;
				if (rec.code != "I_M") throw BuildError("an identity map holds only I_M records, found " + rec.code);
				auto pattern = IdentityPattern::Parse(rec.fields.at(0).s);
				if (!pattern) throw BuildError("identity map: \"" + rec.fields.at(0).s + "\" is not an identity pattern");
				map.entries_.push_back({*pattern, rec.fields.at(1).s, Params::FromFields(rec.fields, 2)});
			}
			return map;
		}

		IdentityMap IdentityMap::Parse(std::string_view text) {
			auto result = Protocol::Parse(text);
			if (!result.Ok())
				throw BuildError("identity map: " + std::string(Protocol::ErrorClassName(result.error->error_class)) + " at byte " +
				                 std::to_string(result.error->offset) + ": " + result.error->message);
			return FromDocument(result.document);
		}

		const IdentityMapping* IdentityMap::Find(const Identity& id) const {
			const IdentityMapping* best = nullptr;
			for (const auto& e : entries_) {
				if (!e.pattern.Matches(id)) continue;
				if (!best || e.pattern.Specificity() > best->pattern.Specificity()) {
					best = &e;
				} else if (e.pattern.Specificity() == best->pattern.Specificity() && !SameMapping(e, *best)) {
					throw BuildError("identity " + id.ToString() + " matches \"" + best->pattern.ToString() + "\" and \"" +
					                 e.pattern.ToString() + "\" equally, with different mappings");
				}
			}
			return best;
		}

		Protocol::Document IdentityMap::ToDocument() const {
			Protocol::Document doc;
			Record version;
			version.kind = '@';
			version.fields = {Field::MakeText(FieldType::Id, "MRS"), Field::MakeNum(0.2)};
			doc.records.push_back(version);
			for (const auto& e : entries_) {
				Record rec;
				rec.kind = 'I';
				rec.code = "I_M";
				rec.fields = {Field::MakeText(FieldType::Str, e.pattern.ToString()), Field::MakeText(FieldType::Id, e.type)};
				for (auto& f : e.params.ToFields()) rec.fields.push_back(std::move(f));
				doc.records.push_back(std::move(rec));
			}
			return doc;
		}

		const char* IdentitySourceName(IdentityMatch::Source source) {
			switch (source) {
			case IdentityMatch::Source::RobotMap: return "robot map";
			case IdentityMatch::Source::Descriptor: return "descriptor";
			case IdentityMatch::Source::Library: return "library";
			case IdentityMatch::Source::Registry: return "registry";
			}
			return "registry";
		}

		Record ParseDescriptor(std::string_view text) {
			auto result = Protocol::Parse(text);
			if (!result.Ok())
				throw BuildError("descriptor: " + std::string(Protocol::ErrorClassName(result.error->error_class)) + " at byte " +
				                 std::to_string(result.error->offset) + ": " + result.error->message);
			const Record* found = nullptr;
			for (const Record& rec : result.document.records) {
				if (rec.kind == '@') continue;
				if (rec.kind != 'D' || rec.code == "D_H") throw BuildError("a descriptor is one D record other than D_H, found " + rec.code);
				if (found) throw BuildError("a descriptor holds one D record");
				found = &rec;
			}
			if (!found) throw BuildError("the descriptor has no D record");
			return *found;
		}

		std::optional<IdentityMatch> IdentityResolver::Resolve(const std::string& identity, const std::string& descriptor) const {
			auto id = Identity::Parse(identity);
			if (!id) return std::nullopt;
			using Source = IdentityMatch::Source;
			auto from_map = [](const IdentityMapping& m, Source source) {
				IdentityMatch match;
				match.source = source;
				match.type = m.type;
				match.params = m.params;
				return match;
			};
			if (robot_)
				if (const auto* m = robot_->Find(*id)) return from_map(*m, Source::RobotMap);
			if (!descriptor.empty()) {
				IdentityMatch match;
				match.source = Source::Descriptor;
				match.descriptor = ParseDescriptor(descriptor);
				match.type = match.descriptor->fields.at(1).s;
				match.params = Params::FromFields(match.descriptor->fields, 2);
				return match;
			}
			for (const IdentityMap* lib : library_)
				if (const auto* m = lib->Find(*id)) return from_map(*m, Source::Library);
			std::string key = registry_.Match(*id);
			if (key.empty()) return std::nullopt;
			IdentityMatch match;
			match.source = Source::Registry;
			match.type = std::move(key);
			return match;
		}
	}
}
