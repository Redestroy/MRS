#pragma once
// Batched 2D drawing for the viewer: thick lines, shaded balls and text from the built-in font,
// so a frame is a handful of draw calls and the viewer needs no font or image files.
#include <string>
#include <vector>

#include <SFML/Graphics.hpp>

namespace MRS {
	namespace Viewer {
		class Canvas {
		public:
			Canvas();

			// Lines, flat shapes and text go to the current layer; Flush draws the layers in order.
			void Line(sf::Vector2f a, sf::Vector2f b, float width, sf::Color ca, sf::Color cb);
			void Line(sf::Vector2f a, sf::Vector2f b, float width, sf::Color c) { Line(a, b, width, c, c); }
			void Triangle(sf::Vector2f a, sf::Vector2f b, sf::Vector2f c, sf::Color colour);
			void Rect(sf::FloatRect r, sf::Color colour);
			void Disc(sf::Vector2f centre, float radius, sf::Color colour, int segments = 20);
			void Ring(sf::Vector2f centre, float radius, float width, sf::Color colour, int segments = 24);
			// A shaded ball; balls are depth sorted (far first) within a flush.
			void Ball(sf::Vector2f centre, float radius, sf::Color colour, double depth);
			void Text(sf::Vector2f top_left, const std::string& text, sf::Color colour);
			static float TextWidth(const std::string& text);
			static float TextHeight();

			// Draws: flat shapes, balls, then text.
			void Flush(sf::RenderTarget& target);

		private:
			struct BallItem {
				sf::Vector2f centre;
				float radius;
				sf::Color colour;
				double depth;
			};
			sf::VertexArray flat_{sf::PrimitiveType::Triangles};
			sf::VertexArray glyphs_{sf::PrimitiveType::Triangles};
			std::vector<BallItem> balls_;
			sf::Texture font_, ball_;
		};
	}
}
