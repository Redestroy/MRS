// MRS dot viewer (spec 17 §4): replays a track file in 3D with the tasks of its task files.
//
// usage: mrs_dot_viewer <track.csv> [task files .mrsl/.mrst/.mrs ...] [options]
//   --trail S        seconds of fading track behind each robot (default 20)
//   --shape sphere|quad   how a robot is drawn (default sphere; quad shows its attitude)
//   --size M         robot radius in metres (default 0.6; never drawn smaller than a few pixels)
//   --speed X        replay speed (default 1)
//   --at S           start at time S
//   --follow NAME    start following a robot (as its number key does)
//   --screenshot F   render one frame at --at to an image file and exit
//   --window WxH     window size (default 1280x800)
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iomanip>
#include <iostream>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <SFML/Graphics.hpp>

#include "Camera.h"
#include "Canvas.h"
#include "mrs/replay/TaskMarkers.h"
#include "mrs/replay/Track.h"

using namespace MRS;
using namespace MRS::Viewer;

namespace {
	const char* kUsage =
	    "usage: mrs_dot_viewer <track.csv> [task files .mrsl/.mrst/.mrs ...] [--trail S] [--shape sphere|quad]\n"
	    "                      [--size M] [--speed X] [--at S] [--follow ROBOT] [--screenshot FILE] [--window WxH]\n";

	const char* kHelp[] = {
	    "Space      play / pause",
	    "Left/Right step 1 s (Shift: 10 s)",
	    "Up/Down    faster / slower",
	    "Home/End   start / end",
	    "[ ]        shorter / longer trail",
	    "1-9  0     follow robot / free camera",
	    "F          fit the scene",
	    "T          view from the top",
	    "L          labels on / off",
	    "G          ground lines on / off",
	    "Q          sphere / quad shape",
	    "H          this help",
	    "Mouse      left: orbit, right: pan,",
	    "           wheel: zoom",
	};

	struct Options {
		std::string track;
		std::vector<std::string> tasks;
		double trail = 20.0, size = 0.6, speed = 1.0;
		std::optional<double> at;
		std::string shape = "sphere", screenshot, follow;
		unsigned width = 1280, height = 800;
	};

	Options Parse(int argc, char** argv) {
		Options o;
		for (int k = 1; k < argc; ++k) {
			const std::string a = argv[k];
			auto next = [&]() -> std::string {
				if (k + 1 >= argc) throw std::runtime_error("missing value after " + a);
				return argv[++k];
			};
			if (a == "--trail") o.trail = std::max(0.0, std::stod(next()));
			else if (a == "--size") o.size = std::max(0.01, std::stod(next()));
			else if (a == "--speed") o.speed = std::stod(next());
			else if (a == "--at") o.at = std::stod(next());
			else if (a == "--shape") o.shape = next();
			else if (a == "--screenshot") o.screenshot = next();
			else if (a == "--follow") o.follow = next();
			else if (a == "--window") {
				const std::string v = next();
				const auto x = v.find('x');
				if (x == std::string::npos) throw std::runtime_error("--window expects WxH");
				o.width = static_cast<unsigned>(std::stoul(v.substr(0, x)));
				o.height = static_cast<unsigned>(std::stoul(v.substr(x + 1)));
			} else if (a == "-h" || a == "--help") throw std::runtime_error("");
			else if (!a.empty() && a[0] == '-') throw std::runtime_error("unknown option " + a);
			else if (o.track.empty()) o.track = a;
			else o.tasks.push_back(a);
		}
		if (o.track.empty()) throw std::runtime_error("no track file");
		if (o.shape != "sphere" && o.shape != "quad") throw std::runtime_error("--shape is sphere or quad");
		return o;
	}

	// Robot colours: distinct, readable on the dark background.
	sf::Color RobotColour(std::size_t k) {
		static const sf::Color c[] = {{78, 160, 255}, {255, 120, 70}, {90, 210, 120}, {230, 90, 200}, {250, 210, 60},
		                              {80, 220, 225}, {255, 150, 170}, {170, 140, 255}, {200, 230, 110}, {255, 180, 100}};
		return c[k % (sizeof(c) / sizeof(c[0]))];
	}
	sf::Color Alpha(sf::Color c, double a) {
		c.a = static_cast<std::uint8_t>(std::clamp(a, 0.0, 1.0) * 255.0);
		return c;
	}

	const sf::Color kBackground{18, 22, 30};
	const sf::Color kGrid{60, 70, 88};
	const sf::Color kText{225, 230, 240};
	const sf::Color kDim{140, 150, 170};
	const sf::Color kOpen{255, 170, 40};
	const sf::Color kDone{80, 200, 110};
	const sf::Color kFailed{235, 70, 70};
	const sf::Color kPanel{10, 13, 20, 215};

	std::string Fixed(double v, int digits) {
		std::ostringstream s;
		s << std::fixed << std::setprecision(digits) << v;
		return s.str();
	}

	Vec3 Rotate(const Replay::Pose& p, Vec3 v) {
		// R = Rz(yaw) Ry(pitch) Rx(roll)
		const double cr = std::cos(p.roll), sr = std::sin(p.roll), cp = std::cos(p.pitch), sp = std::sin(p.pitch), cy = std::cos(p.yaw),
		             sy = std::sin(p.yaw);
		const Vec3 a{v.x, cr * v.y - sr * v.z, sr * v.y + cr * v.z};
		const Vec3 b{cp * a.x + sp * a.z, a.y, -sp * a.x + cp * a.z};
		return {cy * b.x - sy * b.y, sy * b.x + cy * b.y, b.z};
	}

	class App {
	public:
		App(Options o, Replay::Track track, Replay::TaskSet tasks)
		    : o_(std::move(o)), track_(std::move(track)), tasks_(std::move(tasks)) {
			t_ = o_.at ? std::clamp(*o_.at, track_.Start(), track_.End()) : track_.Start();
			speed_ = o_.speed;
			quad_ = o_.shape == "quad";
			if (const auto* m = tasks_.mission ? &*tasks_.mission : (track_.mission ? &*track_.mission : nullptr)) {
				// H_M: id lat lon alt xmin xmax ymin ymax zmax ...
				auto num = [&](std::size_t i) { return m->fields[i].type == Protocol::FieldType::Int ? static_cast<double>(m->fields[i].i) : m->fields[i].n; };
				if (m->fields.size() > 8) fence_ = {num(4), num(5), num(6), num(7), num(8)};
			}
			Bounds();
		}

		int Run() {
			sf::ContextSettings settings;
			settings.antiAliasingLevel = 4;
			sf::RenderWindow window(sf::VideoMode({o_.width, o_.height}), "MRS dot viewer - " + o_.track, sf::Style::Default,
			                        sf::State::Windowed, settings);
			window.setVerticalSyncEnabled(true);
			window.setFramerateLimit(60);
			Fit();
			for (std::size_t k = 0; k < track_.robots.size(); ++k)
				if (track_.robots[k] == o_.follow) {
					follow_ = static_cast<int>(k);
					cam_.distance = 40.0;
				}
			if (!o_.screenshot.empty()) {
				Draw(window);
				sf::Texture shot(window.getSize());
				shot.update(window);
				const bool ok = shot.copyToImage().saveToFile(o_.screenshot);
				if (!ok) std::cerr << "cannot write " << o_.screenshot << "\n";
				return ok ? 0 : 1;
			}
			sf::Clock clock;
			while (window.isOpen()) {
				while (const auto e = window.pollEvent()) Handle(window, *e);
				const double dt = clock.restart().asSeconds();
				if (playing_) {
					t_ += dt * speed_;
					if (t_ >= track_.End()) {
						t_ = track_.End();
						playing_ = false;
					}
				}
				Draw(window);
				window.display();
			}
			return 0;
		}

	private:
		struct Fence {
			double xmin, xmax, ymin, ymax, zmax;
		};

		Options o_;
		Replay::Track track_;
		Replay::TaskSet tasks_;
		std::optional<Fence> fence_;
		Camera cam_;
		Canvas canvas_;
		double t_ = 0.0, speed_ = 1.0;
		bool playing_ = false, labels_ = true, ground_ = true, help_ = false, quad_ = false;
		int follow_ = -1;
		Vec3 lo_{-10, -10, 0}, hi_{10, 10, 10};
		float w_ = 1280, h_ = 800;
		static constexpr float kBar = 54.0f;  // px of the replay bar
		enum class Drag { NONE, ORBIT, PAN, SLIDER } drag_ = Drag::NONE;
		sf::Vector2i last_mouse_;

		// ---- scene extent ------------------------------------------------------------------------
		void Bounds() {
			bool any = false;
			auto add = [&](double x, double y, double z) {
				if (!any) {
					lo_ = hi_ = {x, y, z};
					any = true;
				}
				lo_ = {std::min(lo_.x, x), std::min(lo_.y, y), std::min(lo_.z, z)};
				hi_ = {std::max(hi_.x, x), std::max(hi_.y, y), std::max(hi_.z, z)};
			};
			for (std::size_t i = 0; i < track_.times.size(); ++i)
				for (std::size_t r = 0; r < track_.robots.size(); ++r)
					if (track_.present[i][r]) add(track_.samples[i][r].x, track_.samples[i][r].y, track_.samples[i][r].z);
			for (const auto& m : tasks_.markers) add(m.x, m.y, m.z);
			if (!any) add(0, 0, 0);
			lo_.z = std::min(lo_.z, 0.0);
		}

		void Fit() {
			cam_.target = (lo_ + hi_) * 0.5;
			const Vec3 d = hi_ - lo_;
			const double radius = std::max(5.0, 0.5 * std::sqrt(Dot(d, d)));
			cam_.distance = radius / std::sin(cam_.fov / 2.0) * 0.85;
			follow_ = -1;
		}

		// ---- input --------------------------------------------------------------------------------
		sf::FloatRect SliderRect() const { return {{64.f, h_ - kBar + 22.f}, {std::max(50.f, w_ - 64.f - 300.f), 10.f}}; }
		sf::FloatRect PlayRect() const { return {{14.f, h_ - kBar + 11.f}, {34.f, 32.f}}; }

		void Seek(float x) {
			const auto r = SliderRect();
			const double f = std::clamp((x - r.position.x) / r.size.x, 0.f, 1.f);
			t_ = track_.Start() + f * (track_.End() - track_.Start());
		}

		void Step(double s) { t_ = std::clamp(t_ + s, track_.Start(), track_.End()); }

		void Handle(sf::RenderWindow& window, const sf::Event& e) {
			if (e.is<sf::Event::Closed>()) window.close();
			if (const auto* r = e.getIf<sf::Event::Resized>()) {
				window.setView(sf::View(sf::FloatRect({0.f, 0.f}, {static_cast<float>(r->size.x), static_cast<float>(r->size.y)})));
			}
			if (const auto* k = e.getIf<sf::Event::KeyPressed>()) Key(window, *k);
			if (const auto* b = e.getIf<sf::Event::MouseButtonPressed>()) {
				const sf::Vector2f p{static_cast<float>(b->position.x), static_cast<float>(b->position.y)};
				last_mouse_ = b->position;
				if (b->button == sf::Mouse::Button::Left && PlayRect().contains(p)) Toggle();
				else if (b->button == sf::Mouse::Button::Left && p.y > h_ - kBar) {
					auto r = SliderRect();
					r.position.y -= 12.f;
					r.size.y += 24.f;
					if (r.contains(p)) {
						drag_ = Drag::SLIDER;
						Seek(p.x);
					}
				} else if (b->button == sf::Mouse::Button::Left) drag_ = Drag::ORBIT;
				else if (b->button == sf::Mouse::Button::Right || b->button == sf::Mouse::Button::Middle) drag_ = Drag::PAN;
			}
			if (e.is<sf::Event::MouseButtonReleased>()) drag_ = Drag::NONE;
			if (const auto* m = e.getIf<sf::Event::MouseMoved>()) {
				const double dx = m->position.x - last_mouse_.x, dy = m->position.y - last_mouse_.y;
				last_mouse_ = m->position;
				if (drag_ == Drag::ORBIT) cam_.Orbit(dx, dy);
				if (drag_ == Drag::PAN) {
					cam_.Pan(dx, dy);
					follow_ = -1;
				}
				if (drag_ == Drag::SLIDER) Seek(static_cast<float>(m->position.x));
			}
			if (const auto* s = e.getIf<sf::Event::MouseWheelScrolled>()) cam_.Zoom(s->delta);
		}

		void Toggle() {
			if (!playing_ && t_ >= track_.End()) t_ = track_.Start();
			playing_ = !playing_;
		}

		void Key(sf::RenderWindow& window, const sf::Event::KeyPressed& k) {
			using K = sf::Keyboard::Key;
			static const double speeds[] = {0.125, 0.25, 0.5, 1, 2, 4, 8, 16, 32, 64};
			switch (k.code) {
			case K::Escape: window.close(); break;
			case K::Space: Toggle(); break;
			case K::Left: Step(k.shift ? -10 : -1); break;
			case K::Right: Step(k.shift ? 10 : 1); break;
			case K::Home: t_ = track_.Start(); break;
			case K::End: t_ = track_.End(); break;
			case K::Up:
				for (double s : speeds)
					if (s > speed_ + 1e-9) {
						speed_ = s;
						break;
					}
				break;
			case K::Down:
				for (auto it = std::rbegin(speeds); it != std::rend(speeds); ++it)
					if (*it < speed_ - 1e-9) {
						speed_ = *it;
						break;
					}
				break;
			case K::LBracket: o_.trail = std::max(0.0, o_.trail - (o_.trail > 10 ? 5 : 1)); break;
			case K::RBracket: o_.trail += o_.trail >= 10 ? 5 : 1; break;
			case K::F: Fit(); break;
			case K::T: cam_.pitch = 1.55; break;
			case K::L: labels_ = !labels_; break;
			case K::G: ground_ = !ground_; break;
			case K::Q: quad_ = !quad_; break;
			case K::H: help_ = !help_; break;
			case K::Num0: follow_ = -1; break;
			default:
				if (k.code >= K::Num1 && k.code <= K::Num9) {
					const int r = static_cast<int>(k.code) - static_cast<int>(K::Num1);
					if (r < static_cast<int>(track_.robots.size())) follow_ = r;
				}
				break;
			}
		}

		// ---- drawing ------------------------------------------------------------------------------
		void Line3(Vec3 a, Vec3 b, float width, sf::Color ca, sf::Color cb) {
			sf::Vector2f sa, sb;
			if (cam_.ProjectSegment(a, b, sa, sb)) canvas_.Line(sa, sb, width, ca, cb);
		}
		void Line3(Vec3 a, Vec3 b, float width, sf::Color c) { Line3(a, b, width, c, c); }

		// An ellipse in 3D: centre, two half axes.
		void Ellipse3(Vec3 c, Vec3 u, Vec3 v, float width, sf::Color colour, int segments = 36) {
			Vec3 prev = c + u;
			for (int k = 1; k <= segments; ++k) {
				const double a = 2.0 * Camera::kPi * k / segments;
				const Vec3 p = c + u * std::cos(a) + v * std::sin(a);
				Line3(prev, p, width, colour);
				prev = p;
			}
		}

		void Ground() {
			const double span = std::max({hi_.x - lo_.x, hi_.y - lo_.y, 20.0});
			const double step = span > 400 ? 50 : span > 150 ? 20 : span > 50 ? 10 : 5;
			double x0 = std::floor(lo_.x / step) * step - step, x1 = std::ceil(hi_.x / step) * step + step;
			double y0 = std::floor(lo_.y / step) * step - step, y1 = std::ceil(hi_.y / step) * step + step;
			if (fence_) {
				x0 = std::min(x0, fence_->xmin), x1 = std::max(x1, fence_->xmax);
				y0 = std::min(y0, fence_->ymin), y1 = std::max(y1, fence_->ymax);
			}
			for (double x = x0; x <= x1 + 1e-9; x += step) Line3({x, y0, 0}, {x, y1, 0}, 1.f, Alpha(kGrid, std::fabs(x) < 1e-9 ? 1.0 : 0.6));
			for (double y = y0; y <= y1 + 1e-9; y += step) Line3({x0, y, 0}, {x1, y, 0}, 1.f, Alpha(kGrid, std::fabs(y) < 1e-9 ? 1.0 : 0.6));
			// ENU axes at the origin: x east (red), y north (green), z up (blue).
			const double a = step;
			Line3({0, 0, 0}, {a, 0, 0}, 2.f, {220, 80, 80});
			Line3({0, 0, 0}, {0, a, 0}, 2.f, {80, 200, 90});
			Line3({0, 0, 0}, {0, 0, a}, 2.f, {90, 140, 255});
			if (fence_) {
				const auto& f = *fence_;
				const sf::Color c = Alpha({200, 120, 120}, 0.45);
				const Vec3 p[4] = {{f.xmin, f.ymin, 0}, {f.xmax, f.ymin, 0}, {f.xmax, f.ymax, 0}, {f.xmin, f.ymax, 0}};
				for (int k = 0; k < 4; ++k) {
					const Vec3 a0 = p[k], a1 = p[(k + 1) % 4];
					Line3(a0, a1, 1.f, c);
					Line3(a0 + Vec3{0, 0, f.zmax}, a1 + Vec3{0, 0, f.zmax}, 1.f, c);
					Line3(a0, a0 + Vec3{0, 0, f.zmax}, 1.f, c);
				}
			}
		}

		sf::Color MarkerColour(Replay::MarkerState s) const {
			return s == Replay::MarkerState::DONE ? kDone : s == Replay::MarkerState::FAILED ? kFailed : kOpen;
		}

		// The condition volume, the ball and its drop line (scene pass).
		void MarkerScene(const Replay::TaskMarker& m, Replay::MarkerState s) {
			const sf::Color c = MarkerColour(s);
			const double fade = s == Replay::MarkerState::OPEN ? 1.0 : 0.45;
			const Vec3 p{m.x, m.y, m.z};
			if (ground_) Line3(p, {m.x, m.y, 0}, 1.f, Alpha(c, 0.25 * fade));
			// The tolerance: tol_xy across, tol_z up (a sphere when they are equal).
			if (m.tol_xy > 0) {
				Ellipse3(p, {m.tol_xy, 0, 0}, {0, m.tol_xy, 0}, 1.5f, Alpha(c, 0.8 * fade));
				Ellipse3(p, {m.tol_xy, 0, 0}, {0, 0, std::max(m.tol_z, 0.01)}, 1.f, Alpha(c, 0.5 * fade));
				Ellipse3(p, {0, m.tol_xy, 0}, {0, 0, std::max(m.tol_z, 0.01)}, 1.f, Alpha(c, 0.5 * fade));
				sf::Vector2f sp;
				double depth;
				if (cam_.Project(p, sp, depth)) {
					const float r = static_cast<float>(m.tol_xy * cam_.PixelsPerMetre(depth));
					if (r > 6.f) canvas_.Ball(sp, r, Alpha(c, 0.18 * fade), depth + 1e-3);
				}
			}
			sf::Vector2f sp;
			double depth;
			if (!cam_.Project(p, sp, depth)) return;
			const float r = std::max(4.f, static_cast<float>(0.4 * cam_.PixelsPerMetre(depth)));
			canvas_.Ball(sp, r, Alpha(c, fade), depth);
		}

		// The flag above the ball and its label (overlay pass).
		void MarkerFlag(const Replay::TaskMarker& m, Replay::MarkerState s) {
			sf::Vector2f sp;
			double depth;
			if (!cam_.Project({m.x, m.y, m.z}, sp, depth)) return;
			const sf::Color c = MarkerColour(s);
			const double fade = s == Replay::MarkerState::OPEN ? 1.0 : 0.55;
			const float r = std::max(4.f, static_cast<float>(0.4 * cam_.PixelsPerMetre(depth)));
			const sf::Vector2f top = sp - sf::Vector2f{0.f, r + 22.f};
			canvas_.Line(sp - sf::Vector2f{0.f, r}, top, 1.5f, Alpha({230, 230, 230}, fade));
			canvas_.Triangle(top, top + sf::Vector2f{14.f, 4.5f}, top + sf::Vector2f{0.f, 9.f}, Alpha(c, fade));
			if (!labels_) return;
			std::string label = m.task;
			if (s == Replay::MarkerState::DONE && !m.done_by.empty()) label += " (" + m.done_by + ")";
			if (s == Replay::MarkerState::FAILED) label += " failed";
			canvas_.Text(top + sf::Vector2f{17.f, -3.f}, label, Alpha(kText, fade));
		}

		void Robot(std::size_t k, const Replay::Pose& p) {
			const sf::Color c = RobotColour(k);
			const Vec3 at{p.x, p.y, p.z};
			sf::Vector2f sp;
			double depth;
			if (!cam_.Project(at, sp, depth)) return;
			const double ppm = cam_.PixelsPerMetre(depth);
			// Never smaller than a few pixels, so a robot far away is still seen.
			const double size = std::max(o_.size, 7.0 / ppm);
			if (ground_) {
				Line3(at, {p.x, p.y, 0}, 1.f, Alpha(c, 0.35));
				sf::Vector2f g;
				double gd;
				if (cam_.Project({p.x, p.y, 0}, g, gd)) canvas_.Disc(g, 2.5f, Alpha(c, 0.5), 10);
			}
			if (quad_) {
				// X frame with rotors, nose along body x.
				const double arm = size * 1.6;
				for (double a : {Camera::kPi / 4, 3 * Camera::kPi / 4, 5 * Camera::kPi / 4, 7 * Camera::kPi / 4}) {
					const Vec3 tip = at + Rotate(p, {arm * std::cos(a), arm * std::sin(a), 0});
					Line3(at, tip, 3.f, c);
					Ellipse3(tip, Rotate(p, {arm * 0.45, 0, 0}), Rotate(p, {0, arm * 0.45, 0}), 1.5f, Alpha(c, 0.9), 16);
				}
				Line3(at, at + Rotate(p, {arm * 1.3, 0, 0}), 2.f, kText);
				canvas_.Ball(sp, static_cast<float>(size * 0.5 * ppm), c, depth);
			} else {
				canvas_.Ball(sp, static_cast<float>(size * ppm), c, depth);
			}
			if (labels_) canvas_.Text(sp + sf::Vector2f{static_cast<float>(size * ppm) + 4.f, -Canvas::TextHeight()}, track_.robots[k], c);
		}

		void Trail(std::size_t k, const Replay::Pose& now) {
			if (o_.trail <= 0) return;
			const sf::Color c = RobotColour(k);
			const double from = t_ - o_.trail;
			std::size_t i = track_.Index(std::max(from, track_.Start()));
			const std::size_t end = track_.Index(t_);
			Vec3 prev;
			double prev_t = 0;
			bool have = false;
			auto add = [&](Vec3 p, double t) {
				if (have && t > from) {
					const double a0 = 1.0 - (t_ - std::max(prev_t, from)) / o_.trail, a1 = 1.0 - (t_ - t) / o_.trail;
					Line3(prev, p, 2.f, Alpha(c, 0.85 * a0), Alpha(c, 0.85 * a1));
				}
				prev = p;
				prev_t = t;
				have = true;
			};
			for (; i <= end && i < track_.times.size(); ++i) {
				if (!track_.present[i][k]) {
					have = false;
					continue;
				}
				const auto& s = track_.samples[i][k];
				add({s.x, s.y, s.z}, track_.times[i]);
			}
			add({now.x, now.y, now.z}, t_);
		}

		void Draw(sf::RenderWindow& window) {
			w_ = static_cast<float>(window.getSize().x);
			h_ = static_cast<float>(window.getSize().y);
			cam_.Viewport(w_, h_ - kBar);
			std::vector<std::optional<Replay::Pose>> poses(track_.robots.size());
			for (std::size_t k = 0; k < poses.size(); ++k) {
				Replay::Pose p;
				if (track_.At(k, t_, p)) poses[k] = p;
			}
			if (follow_ >= 0 && poses[static_cast<std::size_t>(follow_)]) {
				const auto& p = *poses[static_cast<std::size_t>(follow_)];
				cam_.target = {p.x, p.y, p.z};
			}

			window.clear(kBackground);
			// Scene: ground, conditions, trails, balls, robot labels.
			Ground();
			for (const auto& m : tasks_.markers) {
				const auto s = m.StateAt(t_);
				if (s != Replay::MarkerState::HIDDEN) MarkerScene(m, s);
			}
			for (std::size_t k = 0; k < poses.size(); ++k)
				if (poses[k]) Trail(k, *poses[k]);
			for (std::size_t k = 0; k < poses.size(); ++k)
				if (poses[k]) Robot(k, *poses[k]);
			canvas_.Flush(window);
			// Flags over the scene.
			for (const auto& m : tasks_.markers) {
				const auto s = m.StateAt(t_);
				if (s != Replay::MarkerState::HIDDEN) MarkerFlag(m, s);
			}
			canvas_.Flush(window);
			Overlay(poses);
			canvas_.Flush(window);
		}

		void Overlay(const std::vector<std::optional<Replay::Pose>>& poses) {
			const float lh = Canvas::TextHeight() + 2.f;
			// Legend: robots, with altitude.
			float y = 10.f;
			const float lw = 200.f;
			canvas_.Rect({{8.f, 6.f}, {lw, 8.f + lh * static_cast<float>(track_.robots.size() + 2)}}, kPanel);
			int open = 0, done = 0, failed = 0;
			for (const auto& m : tasks_.markers) {
				const auto s = m.StateAt(t_);
				open += s == Replay::MarkerState::OPEN;
				done += s == Replay::MarkerState::DONE;
				failed += s == Replay::MarkerState::FAILED;
			}
			canvas_.Text({16.f, y}, "tasks " + std::to_string(done) + " done, " + std::to_string(open) + " open" + (failed ? ", " + std::to_string(failed) + " failed" : ""), kText);
			y += lh * 1.5f;
			for (std::size_t k = 0; k < track_.robots.size(); ++k) {
				const sf::Color c = RobotColour(k);
				canvas_.Disc({22.f, y + lh / 2.f - 1.f}, 5.f, c);
				std::string line = (k < 9 ? std::to_string(k + 1) + " " : "  ") + track_.robots[k];
				if (poses[k]) line += "  z " + Fixed(poses[k]->z, 1) + " m";
				else line += "  -";
				if (static_cast<int>(k) == follow_) line += "  *";
				canvas_.Text({34.f, y}, line, poses[k] ? kText : kDim);
				y += lh;
			}

			// Help.
			if (help_) {
				const float hw = 300.f, hh = 12.f + lh * static_cast<float>(std::size(kHelp));
				canvas_.Rect({{w_ - hw - 8.f, 6.f}, {hw, hh}}, kPanel);
				float hy = 12.f;
				for (const char* line : kHelp) {
					canvas_.Text({w_ - hw + 4.f, hy}, line, kText);
					hy += lh;
				}
			} else {
				canvas_.Text({w_ - Canvas::TextWidth("H: help") - 12.f, 10.f}, "H: help", kDim);
			}

			// Replay bar.
			canvas_.Rect({{0.f, h_ - kBar}, {w_, kBar}}, {10, 13, 20});
			const auto pr = PlayRect();
			canvas_.Rect(pr, {40, 48, 64});
			const sf::Vector2f pc = pr.position + pr.size / 2.f;
			if (playing_) {
				canvas_.Rect({{pc.x - 7.f, pc.y - 8.f}, {5.f, 16.f}}, kText);
				canvas_.Rect({{pc.x + 2.f, pc.y - 8.f}, {5.f, 16.f}}, kText);
			} else {
				canvas_.Triangle({pc.x - 6.f, pc.y - 9.f}, {pc.x + 9.f, pc.y}, {pc.x - 6.f, pc.y + 9.f}, kText);
			}
			const auto sr = SliderRect();
			const double span = std::max(1e-9, track_.End() - track_.Start());
			auto xof = [&](double t) { return sr.position.x + static_cast<float>((t - track_.Start()) / span) * sr.size.x; };
			canvas_.Rect(sr, {45, 52, 68});
			canvas_.Rect({sr.position, {xof(t_) - sr.position.x, sr.size.y}}, {78, 120, 200});
			// Task ticks: dispatch above the slider, done or failed below.
			for (const auto& m : tasks_.markers) {
				if (m.dispatch >= track_.Start() && m.dispatch <= track_.End())
					canvas_.Rect({{xof(m.dispatch) - 0.5f, sr.position.y - 7.f}, {1.5f, 5.f}}, kOpen);
				const std::optional<double> end = m.done ? m.done : m.failed;
				if (end && *end >= track_.Start() && *end <= track_.End())
					canvas_.Rect({{xof(*end) - 0.5f, sr.position.y + sr.size.y + 2.f}, {1.5f, 5.f}}, m.done ? kDone : kFailed);
			}
			canvas_.Disc({xof(t_), sr.position.y + sr.size.y / 2.f}, 8.f, kText);
			const std::string time = Fixed(t_, 1) + " / " + Fixed(track_.End(), 1) + " s   x" + (speed_ < 1 ? Fixed(speed_, 3) : Fixed(speed_, 0)) +
			                         "   trail " + Fixed(o_.trail, 0) + " s";
			canvas_.Text({sr.position.x + sr.size.x + 18.f, h_ - kBar + 18.f}, time, kText);
		}
	};
}

int main(int argc, char** argv) {
	Options o;
	try {
		o = Parse(argc, argv);
	} catch (const std::exception& e) {
		if (*e.what()) std::cerr << "mrs_dot_viewer: " << e.what() << "\n";
		std::cerr << kUsage;
		return 2;
	}
	try {
		Replay::Track track = Replay::ReadTrackFile(o.track);
		if (track.times.empty()) throw std::runtime_error(o.track + ": no rows");
		Replay::TaskSet tasks;
		std::optional<Environment::GeoReference> geo;
		if (track.mission) geo = Replay::GeoOf(*track.mission);
		// A mission header among the task files gives the geo reference when the track has none.
		if (!geo)
			for (const auto& f : o.tasks) {
				Replay::TaskSet probe;
				Replay::AddMarkerFile(f, nullptr, probe);
				if (probe.mission) {
					geo = Replay::GeoOf(*probe.mission);
					break;
				}
			}
		for (const auto& f : o.tasks) Replay::AddMarkerFile(f, geo ? &*geo : nullptr, tasks);
		for (const auto& w : tasks.warnings) std::cerr << "warning: " << w << "\n";
		Replay::ApplyEvents(track.events, tasks);
		std::cout << track.robots.size() << " robots, " << track.times.size() << " rows over " << track.End() - track.Start() << " s, "
		          << tasks.markers.size() << " task markers\n";
		App app(o, std::move(track), std::move(tasks));
		return app.Run();
	} catch (const std::exception& e) {
		std::cerr << "mrs_dot_viewer: " << e.what() << "\n";
		return 1;
	}
}
