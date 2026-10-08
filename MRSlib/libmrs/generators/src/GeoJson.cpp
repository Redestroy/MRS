#include "mrs/generators/GeoJson.h"

#include <cmath>
#include <cstdlib>

namespace MRS {
	namespace Generators {
		const Json& Json::At(const std::string& key) const {
			auto it = fields.find(key);
			if (type != Type::Object || it == fields.end()) throw JsonError("missing key \"" + key + "\"");
			return it->second;
		}

		double Json::Number(const std::string& key, double fallback) const {
			if (!Has(key)) return fallback;
			const Json& v = fields.at(key);
			if (v.type != Type::Number) throw JsonError("\"" + key + "\" is not a number");
			return v.n;
		}

		std::string Json::String(const std::string& key, const std::string& fallback) const {
			if (!Has(key)) return fallback;
			const Json& v = fields.at(key);
			if (v.type != Type::String) throw JsonError("\"" + key + "\" is not a string");
			return v.s;
		}

		namespace {
			class Reader {
			public:
				explicit Reader(const std::string& t) : t_(t) {}

				Json Document() {
					Json v = Value();
					Space();
					if (p_ != t_.size()) Fail("text after the value");
					return v;
				}

			private:
				[[noreturn]] void Fail(const std::string& what) const { throw JsonError(what + " at byte " + std::to_string(p_)); }

				void Space() {
					while (p_ < t_.size() && (t_[p_] == ' ' || t_[p_] == '\t' || t_[p_] == '\n' || t_[p_] == '\r')) ++p_;
				}

				bool Eat(char c) {
					Space();
					if (p_ < t_.size() && t_[p_] == c) {
						++p_;
						return true;
					}
					return false;
				}

				void Expect(char c) {
					if (!Eat(c)) Fail(std::string("expected '") + c + "'");
				}

				bool Word(const char* w) {
					const std::string s(w);
					if (t_.compare(p_, s.size(), s) != 0) return false;
					p_ += s.size();
					return true;
				}

				Json Value() {
					Space();
					if (p_ >= t_.size()) Fail("unexpected end");
					Json v;
					const char c = t_[p_];
					if (c == '{') {
						++p_;
						v.type = Json::Type::Object;
						if (Eat('}')) return v;
						do {
							Space();
							const std::string key = Str();
							Expect(':');
							v.fields[key] = Value();
						} while (Eat(','));
						Expect('}');
					} else if (c == '[') {
						++p_;
						v.type = Json::Type::Array;
						if (Eat(']')) return v;
						do v.items.push_back(Value());
						while (Eat(','));
						Expect(']');
					} else if (c == '"') {
						v.type = Json::Type::String;
						v.s = Str();
					} else if (Word("true")) {
						v.type = Json::Type::Bool;
						v.b = true;
					} else if (Word("false")) {
						v.type = Json::Type::Bool;
					} else if (Word("null")) {
					} else {
						v.type = Json::Type::Number;
						const char* begin = t_.c_str() + p_;
						char* end = nullptr;
						v.n = std::strtod(begin, &end);
						if (end == begin || !std::isfinite(v.n)) Fail("bad value");
						p_ += static_cast<std::size_t>(end - begin);
					}
					return v;
				}

				std::string Str() {
					if (p_ >= t_.size() || t_[p_] != '"') Fail("expected a string");
					++p_;
					std::string out;
					while (true) {
						if (p_ >= t_.size()) Fail("unterminated string");
						const char c = t_[p_++];
						if (c == '"') break;
						if (c != '\\') {
							out += c;
							continue;
						}
						if (p_ >= t_.size()) Fail("unterminated escape");
						const char e = t_[p_++];
						switch (e) {
						case '"': out += '"'; break;
						case '\\': out += '\\'; break;
						case '/': out += '/'; break;
						case 'b': out += '\b'; break;
						case 'f': out += '\f'; break;
						case 'n': out += '\n'; break;
						case 'r': out += '\r'; break;
						case 't': out += '\t'; break;
						case 'u': {
							if (p_ + 4 > t_.size()) Fail("short \\u escape");
							const unsigned long cp = std::strtoul(t_.substr(p_, 4).c_str(), nullptr, 16);
							p_ += 4;
							// UTF-8 for the Basic Multilingual Plane; property text only.
							if (cp < 0x80) out += static_cast<char>(cp);
							else if (cp < 0x800) {
								out += static_cast<char>(0xC0 | (cp >> 6));
								out += static_cast<char>(0x80 | (cp & 0x3F));
							} else {
								out += static_cast<char>(0xE0 | (cp >> 12));
								out += static_cast<char>(0x80 | ((cp >> 6) & 0x3F));
								out += static_cast<char>(0x80 | (cp & 0x3F));
							}
							break;
						}
						default: Fail("bad escape");
						}
					}
					return out;
				}

				const std::string& t_;
				std::size_t p_ = 0;
			};

			Point Position(const Json& p, const Environment::GeoReference& geo) {
				if (p.type != Json::Type::Array || p.items.size() < 2) throw JsonError("a position is [lon, lat] or [lon, lat, alt]");
				for (const auto& x : p.items)
					if (x.type != Json::Type::Number) throw JsonError("a position holds numbers");
				const double lon = p.items[0].n, lat = p.items[1].n, alt = p.items.size() > 2 ? p.items[2].n : 0.0;
				// The altitude is above the reference (spec 11 §4.1), so the ENU z equals it near the origin.
				double lat0 = 0, lon0 = 0, alt0 = 0;
				geo.ToGeodetic({0.0, 0.0, 0.0}, lat0, lon0, alt0);
				const auto e = geo.ToEnu(lat, lon, alt0 + alt);
				return {e.x, e.y, alt};
			}

			std::vector<Point> Positions(const Json& a, const Environment::GeoReference& geo) {
				if (a.type != Json::Type::Array) throw JsonError("coordinates are an array");
				std::vector<Point> out;
				for (const auto& p : a.items) out.push_back(Position(p, geo));
				return out;
			}

			std::vector<Point> Ring(const Json& rings, const Environment::GeoReference& geo) {
				if (rings.type != Json::Type::Array || rings.items.empty()) throw JsonError("a polygon has at least one ring");
				auto ring = Positions(rings.items[0], geo);
				if (ring.size() >= 2 && ring.front()[0] == ring.back()[0] && ring.front()[1] == ring.back()[1]) ring.pop_back();
				if (ring.size() < 3) throw JsonError("a polygon ring has at least 3 corners");
				return ring;
			}

			void AddGeometry(const Json& g, const Json& props, const Environment::GeoReference& geo, std::vector<Feature>& out) {
				const std::string type = g.String("type", "");
				if (type == "GeometryCollection") {
					for (const auto& part : g.At("geometries").items) AddGeometry(part, props, geo, out);
					return;
				}
				const Json& c = g.At("coordinates");
				auto add = [&](const std::string& kind, std::vector<Point> pts) { out.push_back({kind, std::move(pts), props}); };
				if (type == "Point") add("Point", {Position(c, geo)});
				else if (type == "MultiPoint")
					for (const auto& p : c.items) add("Point", {Position(p, geo)});
				else if (type == "LineString") add("LineString", Positions(c, geo));
				else if (type == "MultiLineString")
					for (const auto& l : c.items) add("LineString", Positions(l, geo));
				else if (type == "Polygon") add("Polygon", Ring(c, geo));
				else if (type == "MultiPolygon")
					for (const auto& p : c.items) add("Polygon", Ring(p, geo));
				else throw JsonError("unknown geometry type \"" + type + "\"");
			}
		}

		Json ParseJson(const std::string& text) { return Reader(text).Document(); }

		std::vector<Feature> ReadGeoJson(const std::string& text, const Environment::GeoReference& geo) {
			const Json doc = ParseJson(text);
			std::vector<Feature> out;
			const std::string type = doc.String("type", "");
			Json empty;
			empty.type = Json::Type::Object;
			if (type == "FeatureCollection") {
				for (const auto& f : doc.At("features").items) {
					const Json& props = f.Has("properties") && f.At("properties").type == Json::Type::Object ? f.At("properties") : empty;
					AddGeometry(f.At("geometry"), props, geo, out);
				}
			} else if (type == "Feature") {
				const Json& props = doc.Has("properties") && doc.At("properties").type == Json::Type::Object ? doc.At("properties") : empty;
				AddGeometry(doc.At("geometry"), props, geo, out);
			} else {
				AddGeometry(doc, empty, geo, out);
			}
			return out;
		}
	}
}
