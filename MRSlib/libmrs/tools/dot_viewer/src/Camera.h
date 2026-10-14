#pragma once
// The viewer's orbit camera (spec 17 §4.2): a perspective view of the ENU scene, z up.
#include <algorithm>
#include <cmath>

#include <SFML/System/Vector2.hpp>

namespace MRS {
	namespace Viewer {
		struct Vec3 {
			double x = 0.0, y = 0.0, z = 0.0;
		};
		inline Vec3 operator+(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
		inline Vec3 operator-(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
		inline Vec3 operator*(Vec3 a, double k) { return {a.x * k, a.y * k, a.z * k}; }
		inline double Dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
		inline Vec3 Cross(Vec3 a, Vec3 b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
		inline Vec3 Normal(Vec3 a) {
			const double n = std::sqrt(Dot(a, a));
			return n > 0 ? a * (1.0 / n) : a;
		}

		class Camera {
		public:
			static constexpr double kPi = 3.14159265358979323846;

			Vec3 target;
			double yaw = -kPi / 2;     // the eye's bearing from the target: -π/2 looks north from the south
			double pitch = 0.6;        // rad above the horizon
			double distance = 150.0;   // m
			double fov = 50.0 * kPi / 180.0;  // vertical
			double near_plane = 0.1;

			void Viewport(float width, float height) {
				centre_ = {width / 2.0f, height / 2.0f};
				focal_ = static_cast<double>(height) / 2.0 / std::tan(fov / 2.0);
			}

			Vec3 Eye() const {
				return target + Vec3{std::cos(pitch) * std::cos(yaw), std::cos(pitch) * std::sin(yaw), std::sin(pitch)} * distance;
			}

			// The camera frame: forward, right and up.
			void Frame(Vec3& f, Vec3& r, Vec3& u) const {
				f = Normal(target - Eye());
				r = Normal(Cross(f, {0, 0, 1}));
				u = Cross(r, f);
			}

			// Screen position and depth of a point; false behind the near plane.
			bool Project(Vec3 p, sf::Vector2f& s, double& depth) const {
				Vec3 f, r, u;
				Frame(f, r, u);
				const Vec3 v = p - Eye();
				depth = Dot(v, f);
				if (depth < near_plane) return false;
				s = ToScreen(Dot(v, r), Dot(v, u), depth);
				return true;
			}

			// Both ends of a segment on screen, clipped at the near plane; false when all of it is behind.
			bool ProjectSegment(Vec3 a, Vec3 b, sf::Vector2f& sa, sf::Vector2f& sb) const {
				Vec3 f, r, u;
				Frame(f, r, u);
				const Vec3 e = Eye();
				Vec3 va = a - e, vb = b - e;
				double da = Dot(va, f), db = Dot(vb, f);
				if (da < near_plane && db < near_plane) return false;
				if (da < near_plane || db < near_plane) {
					const double k = (near_plane - da) / (db - da);
					const Vec3 cut = va + (vb - va) * k;
					(da < near_plane ? va : vb) = cut;
					(da < near_plane ? da : db) = near_plane;
				}
				sa = ToScreen(Dot(va, r), Dot(va, u), da);
				sb = ToScreen(Dot(vb, r), Dot(vb, u), db);
				return true;
			}

			double PixelsPerMetre(double depth) const { return focal_ / std::max(depth, near_plane); }
			// Metres per pixel at the target, for panning.
			double MetresPerPixel() const { return distance / focal_; }

			void Orbit(double dx, double dy) {
				yaw -= dx * 0.005;
				pitch = std::clamp(pitch + dy * 0.005, -1.55, 1.55);
			}
			void Pan(double dx, double dy) {
				Vec3 f, r, u;
				Frame(f, r, u);
				const double k = MetresPerPixel();
				target = target - r * (dx * k) + u * (dy * k);
			}
			void Zoom(double steps) { distance = std::clamp(distance * std::pow(0.88, steps), 1.0, 1e5); }

		private:
			sf::Vector2f centre_{400.f, 300.f};
			double focal_ = 600.0;

			sf::Vector2f ToScreen(double x, double y, double depth) const {
				return {centre_.x + static_cast<float>(focal_ * x / depth), centre_.y - static_cast<float>(focal_ * y / depth)};
			}
		};
	}
}
