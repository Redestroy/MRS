#include "mrs/device/Params.h"

#include "mrs/BuildError.h"

namespace MRS {
	namespace Device {
		using Protocol::Field;
		using Protocol::FieldType;

		Params::Params(std::initializer_list<std::pair<std::string, ParamValue>> items) {
			for (const auto& item : items) Add(item.first, item.second);
		}

		Params Params::FromFields(const std::vector<Field>& fields, std::size_t start, std::size_t* next) {
			if (start >= fields.size() || fields[start].type != FieldType::Int) throw BuildError("expected a parameter count");
			const auto count = static_cast<std::size_t>(fields[start].i);
			if (start + 1 + 2 * count > fields.size()) throw BuildError("parameter list is shorter than its count");
			Params p;
			for (std::size_t k = 0; k < count; ++k) {
				const Field& key = fields[start + 1 + 2 * k];
				p.Add(key.s, fields[start + 2 + 2 * k]);
			}
			if (next) *next = start + 1 + 2 * count;
			return p;
		}

		std::vector<Field> Params::ToFields() const {
			std::vector<Field> out;
			out.push_back(Field::MakeInt(static_cast<std::int64_t>(items_.size())));
			for (const auto& item : items_) {
				out.push_back(Field::MakeText(FieldType::Id, item.key));
				out.push_back(item.value);
			}
			return out;
		}

		void Params::Add(std::string key, ParamValue value) { items_.push_back({std::move(key), std::move(value)}); }

		void Params::Set(const std::string& key, ParamValue value) {
			std::vector<Param> kept;
			bool placed = false;
			for (auto& item : items_) {
				if (item.key != key) {
					kept.push_back(std::move(item));
				} else if (!placed) {
					kept.push_back({key, value});
					placed = true;
				}
			}
			if (!placed) kept.push_back({key, std::move(value)});
			items_ = std::move(kept);
		}

		void Params::Merge(const Params& over) {
			std::vector<Param> kept;
			for (auto& item : items_)
				if (!over.Has(item.key)) kept.push_back(std::move(item));
			for (const auto& item : over.items_) kept.push_back(item);
			items_ = std::move(kept);
		}

		bool Params::Has(const std::string& key) const { return Get(key) != nullptr; }

		const ParamValue* Params::Get(const std::string& key) const {
			for (const auto& item : items_)
				if (item.key == key) return &item.value;
			return nullptr;
		}

		std::vector<const ParamValue*> Params::All(const std::string& key) const {
			std::vector<const ParamValue*> out;
			for (const auto& item : items_)
				if (item.key == key) out.push_back(&item.value);
			return out;
		}

		double Params::Num(const std::string& key, double fallback) const {
			const ParamValue* v = Get(key);
			if (!v) return fallback;
			if (v->type == FieldType::Num) return v->n;
			if (v->type == FieldType::Int) return static_cast<double>(v->i);
			throw BuildError("parameter " + key + " must be a number");
		}

		std::int64_t Params::Int(const std::string& key, std::int64_t fallback) const {
			const ParamValue* v = Get(key);
			if (!v) return fallback;
			if (v->type == FieldType::Int) return v->i;
			if (v->type == FieldType::Num && v->n == static_cast<double>(static_cast<std::int64_t>(v->n)))
				return static_cast<std::int64_t>(v->n);
			throw BuildError("parameter " + key + " must be an integer");
		}

		bool Params::Bool(const std::string& key, bool fallback) const {
			const ParamValue* v = Get(key);
			if (!v) return fallback;
			if (v->type == FieldType::Bool) return v->b;
			throw BuildError("parameter " + key + " must be T or F");
		}

		std::string Params::Text(const std::string& key, const std::string& fallback) const {
			const ParamValue* v = Get(key);
			if (!v) return fallback;
			if (v->type == FieldType::Id || v->type == FieldType::Str) return v->s;
			throw BuildError("parameter " + key + " must be an identifier or a string");
		}

		std::vector<std::string> Params::Texts(const std::string& key) const {
			std::vector<std::string> out;
			for (const ParamValue* v : All(key)) {
				if (v->type != FieldType::Id && v->type != FieldType::Str && v->type != FieldType::Code)
					throw BuildError("parameter " + key + " must be an identifier, a code or a string");
				out.push_back(v->s);
			}
			return out;
		}

		ParamValue NumValue(double v) { return Field::MakeNum(v); }
		ParamValue BoolValue(bool v) { return Field::MakeBool(v); }
		ParamValue IdValue(std::string v) { return Field::MakeText(FieldType::Id, std::move(v)); }
		ParamValue StrValue(std::string v) { return Field::MakeText(FieldType::Str, std::move(v)); }

		std::string ValueText(const ParamValue& v) {
			return v.type == FieldType::Id || v.type == FieldType::Str ? v.s : std::string();
		}
	}
}
