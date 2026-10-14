#include "Canvas.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "FontData.h"

namespace MRS {
	namespace Viewer {
		namespace {
			constexpr float kPi = 3.14159265f;
			constexpr unsigned kBall = 64;  // px of the ball texture

			void Quad(sf::VertexArray& v, sf::Vector2f p0, sf::Vector2f p1, sf::Vector2f p2, sf::Vector2f p3, sf::Color c0, sf::Color c1,
			          sf::Color c2, sf::Color c3, sf::Vector2f t0 = {}, sf::Vector2f t1 = {}, sf::Vector2f t2 = {}, sf::Vector2f t3 = {}) {
				v.append({p0, c0, t0});
				v.append({p1, c1, t1});
				v.append({p2, c2, t2});
				v.append({p0, c0, t0});
				v.append({p2, c2, t2});
				v.append({p3, c3, t3});
			}

			// White ball lit from the upper left, with a soft edge; tinted by the vertex colour.
			sf::Image BallImage() {
				std::vector<std::uint8_t> px(kBall * kBall * 4);
				const float r = kBall / 2.0f - 1.0f;
				for (unsigned y = 0; y < kBall; ++y)
					for (unsigned x = 0; x < kBall; ++x) {
						const float dx = (static_cast<float>(x) + 0.5f - kBall / 2.0f) / r;
						const float dy = (static_cast<float>(y) + 0.5f - kBall / 2.0f) / r;
						const float d2 = dx * dx + dy * dy;
						std::uint8_t* p = &px[(y * kBall + x) * 4];
						if (d2 > 1.0f) {
							p[0] = p[1] = p[2] = 255;
							p[3] = 0;
							continue;
						}
						const float nz = std::sqrt(1.0f - d2);
						const float light = std::max(0.0f, -0.45f * dx - 0.55f * dy + 0.70f * nz);
						const float shade = 0.35f + 0.65f * light;
						const float spec = std::pow(std::max(0.0f, light), 24.0f) * 0.6f;
						const float edge = std::clamp((1.0f - std::sqrt(d2)) * r, 0.0f, 1.0f);
						const auto c = static_cast<std::uint8_t>(std::clamp(255.0f * (shade + spec), 0.0f, 255.0f));
						p[0] = p[1] = p[2] = c;
						p[3] = static_cast<std::uint8_t>(255.0f * edge);
					}
				return sf::Image({kBall, kBall}, px.data());
			}

			sf::Image FontImage() {
				using namespace FontData;
				const unsigned w = kGlyphW * kCount;
				std::vector<std::uint8_t> px(w * kGlyphH * 4);
				for (unsigned k = 0; k < w * kGlyphH; ++k) {
					px[k * 4 + 0] = px[k * 4 + 1] = px[k * 4 + 2] = 255;
					px[k * 4 + 3] = kAlpha[k];
				}
				return sf::Image({w, kGlyphH}, px.data());
			}
		}

		Canvas::Canvas() : font_(FontImage()), ball_(BallImage()) {
			ball_.setSmooth(true);
		}

		void Canvas::Line(sf::Vector2f a, sf::Vector2f b, float width, sf::Color ca, sf::Color cb) {
			sf::Vector2f d = b - a;
			const float len = std::sqrt(d.x * d.x + d.y * d.y);
			if (len < 1e-3f) return;
			const sf::Vector2f n{-d.y / len * width / 2.0f, d.x / len * width / 2.0f};
			Quad(flat_, a + n, b + n, b - n, a - n, ca, cb, cb, ca);
		}

		void Canvas::Triangle(sf::Vector2f a, sf::Vector2f b, sf::Vector2f c, sf::Color colour) {
			flat_.append({a, colour});
			flat_.append({b, colour});
			flat_.append({c, colour});
		}

		void Canvas::Rect(sf::FloatRect r, sf::Color colour) {
			const sf::Vector2f p = r.position, s = r.size;
			Quad(flat_, p, {p.x + s.x, p.y}, p + s, {p.x, p.y + s.y}, colour, colour, colour, colour);
		}

		void Canvas::Disc(sf::Vector2f c, float radius, sf::Color colour, int segments) {
			for (int k = 0; k < segments; ++k) {
				const float a0 = 2 * kPi * static_cast<float>(k) / static_cast<float>(segments);
				const float a1 = 2 * kPi * static_cast<float>(k + 1) / static_cast<float>(segments);
				Triangle(c, c + sf::Vector2f{std::cos(a0), std::sin(a0)} * radius, c + sf::Vector2f{std::cos(a1), std::sin(a1)} * radius, colour);
			}
		}

		void Canvas::Ring(sf::Vector2f c, float radius, float width, sf::Color colour, int segments) {
			for (int k = 0; k < segments; ++k) {
				const float a0 = 2 * kPi * static_cast<float>(k) / static_cast<float>(segments);
				const float a1 = 2 * kPi * static_cast<float>(k + 1) / static_cast<float>(segments);
				Line(c + sf::Vector2f{std::cos(a0), std::sin(a0)} * radius, c + sf::Vector2f{std::cos(a1), std::sin(a1)} * radius, width, colour);
			}
		}

		void Canvas::Ball(sf::Vector2f centre, float radius, sf::Color colour, double depth) { balls_.push_back({centre, radius, colour, depth}); }

		float Canvas::TextWidth(const std::string& text) { return static_cast<float>(text.size() * FontData::kGlyphW); }
		float Canvas::TextHeight() { return static_cast<float>(FontData::kGlyphH); }

		void Canvas::Text(sf::Vector2f p, const std::string& text, sf::Color colour) {
			using namespace FontData;
			p = {std::round(p.x), std::round(p.y)};
			const float w = static_cast<float>(kGlyphW), h = static_cast<float>(kGlyphH);
			for (char ch : text) {
				const unsigned c = static_cast<unsigned char>(ch);
				if (c > kFirst && c < kFirst + kCount) {
					const float u = static_cast<float>((c - kFirst) * kGlyphW);
					Quad(glyphs_, p, {p.x + w, p.y}, {p.x + w, p.y + h}, {p.x, p.y + h}, colour, colour, colour, colour, {u, 0}, {u + w, 0}, {u + w, h},
					     {u, h});
				}
				p.x += w;
			}
		}

		void Canvas::Flush(sf::RenderTarget& target) {
			target.draw(flat_);
			flat_.clear();

			std::sort(balls_.begin(), balls_.end(), [](const BallItem& a, const BallItem& b) { return a.depth > b.depth; });
			sf::VertexArray balls(sf::PrimitiveType::Triangles);
			const float t = static_cast<float>(kBall);
			for (const auto& b : balls_) {
				const sf::Vector2f c = b.centre;
				const float r = b.radius;
				Quad(balls, {c.x - r, c.y - r}, {c.x + r, c.y - r}, {c.x + r, c.y + r}, {c.x - r, c.y + r}, b.colour, b.colour, b.colour, b.colour, {0, 0},
				     {t, 0}, {t, t}, {0, t});
			}
			balls_.clear();
			sf::RenderStates s;
			s.texture = &ball_;
			target.draw(balls, s);

			s.texture = &font_;
			target.draw(glyphs_, s);
			glyphs_.clear();
		}
	}
}
