#include "mrs/world/View.h"

namespace MRS {
	namespace Environment {
		using Protocol::Field;
		using Protocol::FieldType;

		Protocol::Record ToRecord(const View& view) {
			Protocol::Record rec;
			rec.kind = 'V';
			rec.code = view.code;
			rec.fields.push_back(Field::MakeNum(view.stamp));
			if (view.code == "V_PEER") {
				rec.fields.push_back(Field::MakeInt(static_cast<std::int64_t>(view.values.at(0))));
				for (std::size_t k = 1; k < view.values.size(); ++k) rec.fields.push_back(Field::MakeNum(view.values[k]));
				rec.fields.push_back(Field::MakeText(FieldType::TaskId, view.text.empty() ? "0" : view.text));
			} else if (view.code == "V_REL3") {
				rec.fields.push_back(Field::MakeInt(static_cast<std::int64_t>(view.values.at(0))));
				for (std::size_t k = 1; k < view.values.size(); ++k) rec.fields.push_back(Field::MakeNum(view.values[k]));
			} else if (view.code == "V_DET" || view.code == "V_FLD") {
				rec.fields.push_back(Field::MakeText(FieldType::Id, view.text));
				for (double v : view.values) rec.fields.push_back(Field::MakeNum(v));
			} else {
				for (double v : view.values) rec.fields.push_back(Field::MakeNum(v));
			}
			return rec;
		}

		std::optional<View> ViewFromRecord(const Protocol::Record& rec) {
			if (rec.kind != 'V' || rec.fields.empty()) return std::nullopt;
			View v;
			v.code = rec.code;
			v.stamp = rec.fields[0].n;
			for (std::size_t k = 1; k < rec.fields.size(); ++k) {
				const Field& f = rec.fields[k];
				switch (f.type) {
				case FieldType::Num: v.values.push_back(f.n); break;
				case FieldType::Int: v.values.push_back(static_cast<double>(f.i)); break;
				case FieldType::Id:
				case FieldType::TaskId:
				case FieldType::Str: v.text = f.s; break;
				default: break;
				}
			}
			return v;
		}
	}
}
