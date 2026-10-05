#include "mrs/protocol/Writer.h"

#include <charconv>
#include <map>
#include <stdexcept>

namespace MRS {
	namespace Protocol {
		std::string FormatNumber(double value) {
			char buf[64];
			auto res = std::to_chars(buf, buf + sizeof buf, value);
			if (res.ec != std::errc()) throw std::runtime_error("FormatNumber failed");
			return std::string(buf, res.ptr);
		}

		namespace {
			std::string FormatInt(std::int64_t value) {
				char buf[32];
				auto res = std::to_chars(buf, buf + sizeof buf, value);
				return std::string(buf, res.ptr);
			}

			std::string Quote(const std::string& s) {
				std::string out = "\"";
				for (char c : s) {
					if (c == '"' || c == '\\') out += '\\';
					out += c;
				}
				out += '"';
				return out;
			}

			void WriteRecord(const Record& rec, const std::string& header, const WriteOptions& opt, std::string& out) {
				out += header;
				out += ": ";
				bool first = true;
				auto put = [&](const std::string& token) {
					if (!first) out += ' ';
					out += token;
					first = false;
				};
				if (!rec.code.empty()) put(rec.code);

				// Labels are numbered 1, 2, ... per kind in reference order (spec 01 §3.5).
				std::map<char, int> counter;
				std::vector<std::string> child_label(rec.children.size());
				const auto& f = rec.fields;
				for (std::size_t i = 0; i < f.size();) {
					const Field& field = f[i];
					if (field.type == FieldType::Ref) {
						const char kind = rec.children.at(field.ref).kind;
						std::size_t j = i;
						while (j < f.size() && f[j].type == FieldType::Ref && rec.children.at(f[j].ref).kind == kind) ++j;
						const int start = counter[kind] + 1;
						for (std::size_t k = i; k < j; ++k)
							child_label[f[k].ref] = std::string(1, kind) + "_" + std::to_string(++counter[kind]);
						const int stop = counter[kind];
						const std::string prefix = std::string(1, kind) + "_";
						if (j - i >= 3) {
							put(prefix + std::to_string(start) + ".." + std::to_string(stop));
						} else {
							for (int n = start; n <= stop; ++n) put(prefix + std::to_string(n));
						}
						i = j;
						continue;
					}
					switch (field.type) {
					case FieldType::Int: put(FormatInt(field.i)); break;
					case FieldType::Num: put(FormatNumber(field.n)); break;
					case FieldType::Bool: put(field.b ? "T" : "F"); break;
					case FieldType::Str: put(Quote(field.s)); break;
					default: put(field.s); break;
					}
					++i;
				}
				out += '/';
				if (!opt.compact) out += '\n';
				for (std::size_t c = 0; c < rec.children.size(); ++c)
					WriteRecord(rec.children[c], child_label[c], opt, out);
			}
		}

		std::string Write(const Record& top_level, const WriteOptions& options) {
			std::string out;
			WriteRecord(top_level, std::string(1, top_level.kind), options, out);
			return out;
		}

		std::string Write(const Document& document, const WriteOptions& options) {
			std::string out;
			for (std::size_t i = 0; i < document.records.size(); ++i) {
				if (i > 0 && !options.compact) out += '\n';
				out += Write(document.records[i], options);
			}
			return out;
		}
	}
}
