#include "mrs/protocol/Record.h"

#include <cstring>

namespace MRS {
	namespace Protocol {
		Field Field::MakeInt(std::int64_t v) {
			Field f;
			f.type = FieldType::Int;
			f.i = v;
			return f;
		}

		Field Field::MakeNum(double v) {
			Field f;
			f.type = FieldType::Num;
			f.n = v;
			return f;
		}

		Field Field::MakeBool(bool v) {
			Field f;
			f.type = FieldType::Bool;
			f.b = v;
			return f;
		}

		Field Field::MakeText(FieldType t, std::string v) {
			Field f;
			f.type = t;
			f.s = std::move(v);
			return f;
		}

		Field Field::MakeRef(std::size_t child) {
			Field f;
			f.type = FieldType::Ref;
			f.ref = child;
			return f;
		}

		bool operator==(const Field& a, const Field& b) {
			if (a.type != b.type) return false;
			switch (a.type) {
			case FieldType::Int: return a.i == b.i;
			case FieldType::Num: return std::memcmp(&a.n, &b.n, sizeof a.n) == 0;  // keeps -0 apart from 0
			case FieldType::Bool: return a.b == b.b;
			case FieldType::Ref: return a.ref == b.ref;
			default: return a.s == b.s;
			}
		}

		bool operator==(const Record& a, const Record& b) {
			return a.kind == b.kind && a.code == b.code && a.fields == b.fields && a.children == b.children;
		}

		bool operator==(const Document& a, const Document& b) { return a.records == b.records; }
	}
}
