#pragma once
// Device and port parameters: `n × (id key, value)` slots (spec 04 §2.1).
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "mrs/protocol/Record.h"

namespace MRS {
	namespace Device {
		// A value is a number, boolean, identifier or string, kept as a protocol field.
		using ParamValue = Protocol::Field;

		struct Param {
			std::string key;
			ParamValue value;
		};

		class Params {
		public:
			Params() = default;
			Params(std::initializer_list<std::pair<std::string, ParamValue>> items);

			// Reads `n` and the n pairs from `fields`, starting at `start`. `next` gets the index after them.
			static Params FromFields(const std::vector<Protocol::Field>& fields, std::size_t start, std::size_t* next = nullptr);
			// The count followed by the pairs, ready to put in a record.
			std::vector<Protocol::Field> ToFields() const;

			void Add(std::string key, ParamValue value);
			// Replaces every value of `key` with one value.
			void Set(const std::string& key, ParamValue value);
			// Keys present in `over` replace all values of that key here.
			void Merge(const Params& over);

			bool Has(const std::string& key) const;
			const ParamValue* Get(const std::string& key) const;  // the first value
			std::vector<const ParamValue*> All(const std::string& key) const;
			const std::vector<Param>& Items() const { return items_; }
			bool Empty() const { return items_.empty(); }

			// Typed reads with a default for a missing key. Text accepts identifiers and strings.
			double Num(const std::string& key, double fallback) const;
			std::int64_t Int(const std::string& key, std::int64_t fallback) const;
			bool Bool(const std::string& key, bool fallback) const;
			std::string Text(const std::string& key, const std::string& fallback = {}) const;
			std::vector<std::string> Texts(const std::string& key) const;

		private:
			std::vector<Param> items_;
		};

		ParamValue NumValue(double v);
		ParamValue BoolValue(bool v);
		ParamValue IdValue(std::string v);
		ParamValue StrValue(std::string v);
		// Text of an identifier or string value; empty for other types.
		std::string ValueText(const ParamValue& v);
	}
}
