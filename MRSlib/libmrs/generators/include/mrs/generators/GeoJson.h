#pragma once
// Operator input for the task generators (plan §9.2, spec 11 §4): a small JSON reader and the
// GeoJSON features it carries, converted to the mission's ENU frame.
#include <array>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "mrs/world/GeoReference.h"

namespace MRS {
	namespace Generators {
		struct JsonError : std::runtime_error {
			using std::runtime_error::runtime_error;
		};

		// A JSON value (RFC 8259). Numbers are doubles.
		struct Json {
			enum class Type { Null, Bool, Number, String, Array, Object } type = Type::Null;
			bool b = false;
			double n = 0.0;
			std::string s;
			std::vector<Json> items;             // Array
			std::map<std::string, Json> fields;  // Object

			bool Has(const std::string& key) const { return type == Type::Object && fields.count(key) > 0; }
			const Json& At(const std::string& key) const;  // throws JsonError
			double Number(const std::string& key, double fallback) const;
			std::string String(const std::string& key, const std::string& fallback) const;
		};

		// Parses a JSON text. Throws JsonError with the byte offset.
		Json ParseJson(const std::string& text);

		using Point = std::array<double, 3>;  // ENU x, y, z (m)

		struct Feature {
			std::string geometry;               // Point, LineString, Polygon
			std::vector<Point> points;          // Point: one; LineString: the line; Polygon: the outer ring, not closed
			Json properties;                    // the feature's properties object
		};

		// The features of a GeoJSON Feature, FeatureCollection or bare geometry. Positions are
		// [lon, lat] or [lon, lat, alt] (RFC 7946) and become ENU through the geo reference; a
		// missing altitude is 0 m above the reference. MultiPoint, MultiLineString and
		// MultiPolygon give one feature per part. Polygon holes are ignored. Throws JsonError.
		std::vector<Feature> ReadGeoJson(const std::string& text, const Environment::GeoReference& geo);
	}
}
