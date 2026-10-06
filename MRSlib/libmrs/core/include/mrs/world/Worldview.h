#pragma once
// The worldview (spec 05): named, typed, time-stamped fields with freshness, time series,
// source selection for offered fields, semantic objects and the geo reference.
#include <limits>
#include <map>
#include <optional>
#include <string>
#include <variant>
#include <vector>

#include "mrs/world/GeoReference.h"
#include "mrs/world/TimeSeries.h"

namespace MRS {
	namespace Environment {
		using FieldValue = std::variant<double, bool, std::string>;  // scalar, bool, id

		struct WorldFieldEntry {
			FieldValue value;
			double stamp = 0.0;  // mission time of the information
			bool valid = true;
			std::string source;  // who wrote it; for an offered field, the selected source (spec 05 §5.1)
		};

		// Something in the world (spec 05 §5.4).
		struct SemanticObject {
			std::string id;   // "r2", "det.person.1"
			std::string cls;  // self, peer, target, obstacle, detection
			Enu position;
			Enu velocity;
			double stamp = 0.0;  // last seen
			std::string source;
			double confidence = 1.0;
		};

		class Worldview {
		public:
			static constexpr double kNoMaxAge = std::numeric_limits<double>::infinity();

			Worldview();

			// Writers. A vector field is stored as its components (pose.enu.x, .y, .z).
			// Every scalar write also goes into the path's time series.
			void SetScalar(const std::string& path, double value, double stamp, const std::string& source = {});
			void SetBool(const std::string& path, bool value, double stamp, const std::string& source = {});
			void SetId(const std::string& path, const std::string& value, double stamp, const std::string& source = {});
			void SetVec3(const std::string& path, double x, double y, double z, double stamp, const std::string& source = {});
			// Named components: SetComponents("geo.position", {"lat", "lon", "alt"}, {...}, stamp).
			void SetComponents(const std::string& path, const std::vector<std::string>& names, const std::vector<double>& values,
			                   double stamp, const std::string& source = {});
			void Invalidate(const std::string& path, double stamp);  // valid = false (spec 05 §3)
			void Erase(const std::string& path);                     // the field becomes missing

			// Readers. Each returns nullopt when the field is missing, invalid, stale at
			// mission time t, or of another type.
			std::optional<double> Scalar(const std::string& path, double t) const;
			std::optional<bool> Bool(const std::string& path, double t) const;
			std::optional<std::string> Id(const std::string& path, double t) const;

			// A scalar or a group: TRUE when the field, or every component under it, is
			// present and at most max_age old (the field's default max_age when max_age < 0).
			bool IsFresh(const std::string& path, double t, double max_age = -1.0) const;
			bool Has(const std::string& path) const;

			// Raw access, without freshness, for binding and restoring target fields.
			std::optional<WorldFieldEntry> Raw(const std::string& path) const;
			void Restore(const std::string& path, const std::optional<WorldFieldEntry>& entry);

			// Paths that start with prefix (for example "det.person.").
			std::vector<std::string> PathsWithPrefix(const std::string& prefix) const;

			// Offered fields (spec 05 §5.1). A source offers a value; the worldview writes the field from
			// the best fresh source by the field's source order. Components as for SetComponents;
			// empty names for a scalar.
			void Offer(const std::string& path, const std::string& source, const std::vector<std::string>& names,
			           const std::vector<double>& values, double stamp);
			void OfferScalar(const std::string& path, const std::string& source, double value, double stamp);
			void OfferVec3(const std::string& path, const std::string& source, double x, double y, double z, double stamp);
			void SetSourceOrder(const std::string& path, std::vector<std::string> sources);
			const std::vector<std::string>& SourceOrder(const std::string& path) const;
			// Re-selects every offered field at time t (end of each tick).
			void RefreshSources(double t);
			// The source an offered field is written from now; empty if none.
			std::string SelectedSource(const std::string& path) const;
			std::vector<std::string> OfferedSources(const std::string& path) const;

			// Time series of a scalar path or component; null if it was never written.
			const TimeSeries* History(const std::string& path) const;
			void SetHistoryCapacity(std::size_t capacity) { history_capacity_ = capacity; }

			// Semantic objects (spec 05 §5.4).
			void PutObject(SemanticObject object);
			const SemanticObject* Object(const std::string& id) const;
			const std::map<std::string, SemanticObject>& Objects() const { return objects_; }

			// The team's ENU origin (spec 06 §6); unset until the mission header arrives.
			void SetGeoReference(const GeoReference& geo, double alt0) {
				geo_ = geo;
				alt0_ = alt0;
			}
			const GeoReference* Geo() const { return geo_ ? &*geo_ : nullptr; }
			double GeoAltitude() const { return alt0_; }

			// Default max_age per field (spec 05 §4.1). A component inherits the max_age of
			// its vector field; fields not in the table never go stale.
			void SetMaxAge(const std::string& path, double max_age);
			double MaxAge(const std::string& path) const;

		private:
			const WorldFieldEntry* Fresh(const std::string& path, double t) const;

			struct Candidate {
				std::vector<std::string> names;
				std::vector<double> values;
				double stamp = 0.0;
			};
			void Select(const std::string& path, double t);

			std::map<std::string, WorldFieldEntry> fields_;
			std::map<std::string, double> max_age_;
			std::map<std::string, TimeSeries> history_;
			std::size_t history_capacity_ = 256;
			std::map<std::string, std::map<std::string, Candidate>> offers_;
			std::map<std::string, std::vector<std::string>> source_order_;
			std::map<std::string, std::string> selected_;
			std::map<std::string, SemanticObject> objects_;
			std::optional<GeoReference> geo_;
			double alt0_ = 0.0;
		};
	}
}
