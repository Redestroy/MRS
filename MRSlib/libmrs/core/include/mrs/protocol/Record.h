#pragma once
// Generic object model of the MRS text protocol (spec 01).
// Every record is kept as its code plus typed slots; references to sub-records
// are slots of type Ref that point into the record's children, in reference order.
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace MRS {
	namespace Protocol {
		enum class FieldType { Int, Num, Bool, Id, TaskId, Str, Code, Ref };

		struct Field {
			FieldType type = FieldType::Int;
			std::int64_t i = 0;   // Int
			double n = 0.0;       // Num
			bool b = false;       // Bool
			std::string s;        // Id, TaskId, Str (unescaped), Code
			std::size_t ref = 0;  // Ref: index into Record::children
			std::size_t offset = 0;  // byte offset in the source text; not compared

			static Field MakeInt(std::int64_t v);
			static Field MakeNum(double v);
			static Field MakeBool(bool v);
			static Field MakeText(FieldType t, std::string v);
			static Field MakeRef(std::size_t child);
		};

		struct Record {
			char kind = 0;          // 'T', 'C', '?', '@', ...
			std::string code;       // "T_A"; empty for '?' and '@' (literal bodies)
			std::vector<Field> fields;
			std::vector<Record> children;  // sub-records, in reference order
			std::size_t offset = 0;        // byte offset of the header; not compared
			std::size_t code_offset = 0;   // byte offset of the code token; not compared

			// Kind letter of child i, which equals children[i].kind.
			const Record& Child(std::size_t i) const { return children.at(i); }
		};

		struct Document {
			std::vector<Record> records;  // top-level records in file order
		};

		// Structural equality: same kinds, codes, slot types and values, and children.
		// Offsets and the label numbers used in the source are ignored.
		bool operator==(const Field& a, const Field& b);
		bool operator==(const Record& a, const Record& b);
		bool operator==(const Document& a, const Document& b);
		inline bool operator!=(const Field& a, const Field& b) { return !(a == b); }
		inline bool operator!=(const Record& a, const Record& b) { return !(a == b); }
		inline bool operator!=(const Document& a, const Document& b) { return !(a == b); }
	}
}
