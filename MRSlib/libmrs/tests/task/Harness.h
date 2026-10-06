#pragma once
// Test harness for WP1: a scripted action sink and a toy point-mass UAV that writes the
// worldview fields the UAV behaviours read.
#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "mrs/task/BehaviourLibrary.h"
#include "mrs/task/TaskExecutor.h"
#include "mrs/task/TaskFactory.h"

namespace MRS {
	namespace Test {
		inline std::unique_ptr<Task::Task> OneTask(const Task::TaskFactory& factory, const std::string& text) {
			auto tasks = factory.BuildTasks(text);
			return std::move(tasks.at(0));
		}

		// Records every dispatch and answers with `respond` (DONE by default).
		struct ScriptedSink : Task::IActionSink {
			struct Entry {
				double t;
				Device::ActionMap action;
			};
			std::vector<Entry> log;
			std::function<Device::ActionStatus(const Device::ActionMap&, double)> respond;

			Device::ActionStatus Dispatch(const Device::ActionMap& a, double t) override {
				log.push_back({t, a});
				return respond ? respond(a, t) : Device::ActionStatus::DONE;
			}
			const std::string& LastCode() const { return log.back().action.entries.at(0).action.code; }
		};

		// A toy UAV: takes off, holds position setpoints and lands, with speed limits.
		struct ToyUav : Task::IActionSink {
			double x = 0, y = 0, z = 0, yaw = 0;
			double sx = 0, sy = 0, sz = 0, syaw = 0;
			bool armed = false, landing = false;
			double takeoff_alt = 0;
			double max_xy = 5.0, max_z = 2.0;
			double layer_alt = 20.0;
			unsigned led_mask = 0, led_colour = 0;
			double max_alt = 0;
			std::vector<std::string> codes;  // every code dispatched, in order

			bool Airborne() const { return z > 0.3 && armed; }
			bool Landed() const { return z < 0.15; }

			void Write(Environment::Worldview& w, double t) const {
				w.SetVec3("pose.enu", x, y, z, t);
				w.SetScalar("alt.agl", z, t);
				w.SetScalar("att.yaw", yaw, t);
				w.SetBool("airborne", Airborne(), t);
				w.SetBool("landed", Landed(), t);
				w.SetBool("battery.critical", false, t);
				w.SetScalar("layer.alt", layer_alt, t);
			}

			Device::ActionStatus Dispatch(const Device::ActionMap& m, double) override {
				for (const auto& e : m.entries) {
					const auto& a = e.action;
					codes.push_back(a.code);
					const auto info = Device::FindAction(a.code);
					if (!info) return Device::ActionStatus::REJECTED;
					if (Device::IsIntegerLayout(info->layout)) {
						auto v = Device::UnpackIntegers(info->layout, a.arg);
						if (a.code == "A_L") {
							led_mask = static_cast<unsigned>(v[0]);
							led_colour = static_cast<unsigned>(v[1]);
						}
						continue;
					}
					auto v = Device::UnpackReals(info->layout, a.arg);
					if (a.code == "A_TO") {
						armed = true;
						landing = false;
						takeoff_alt = v[0];
						sx = x;
						sy = y;
						sz = v[0];
						if (z < v[0] - 0.05) return Device::ActionStatus::RUNNING;
					} else if (a.code == "A_LD") {
						landing = true;
						sz = 0;
						if (!Landed()) return Device::ActionStatus::RUNNING;
						armed = false;
					} else if (a.code == "A_PXY") {
						sx = v[0];
						sy = v[1];
					} else if (a.code == "A_PZY") {
						sz = v[0];
						syaw = v[1];
					} else if (a.code == "A_HD") {
						sx = x;
						sy = y;
						sz = z;
					}
				}
				return Device::ActionStatus::DONE;
			}

			void Step(double dt) {
				if (!armed) return;
				const double dx = sx - x, dy = sy - y, d = std::hypot(dx, dy);
				const double step = std::min(d, max_xy * dt);
				if (d > 1e-9) {
					x += dx / d * step;
					y += dy / d * step;
				}
				const double dz = sz - z;
				z += std::clamp(dz, -max_z * dt, max_z * dt);
				yaw = syaw;
				max_alt = std::max(max_alt, z);
			}
		};

		// Runs ticks until the executor is empty or `max_ticks` have passed.
		template <class Before, class After>
		std::vector<Task::TaskEvent> RunUntilEmpty(Task::TaskExecutor& ex, Environment::Worldview& w, double& t, double dt,
		                                           int max_ticks, Before before, After after) {
			std::vector<Task::TaskEvent> events;
			for (int i = 0; i < max_ticks && !ex.Empty(); ++i) {
				before(t);
				auto r = ex.Tick(w, t);
				events.insert(events.end(), r.events.begin(), r.events.end());
				after(t, r);
				t += dt;
			}
			return events;
		}

		inline const Task::TaskEvent* FinalEvent(const std::vector<Task::TaskEvent>& events, const std::string& id) {
			const Task::TaskEvent* last = nullptr;
			for (const auto& e : events)
				if (e.task_id == id) last = &e;
			return last;
		}
	}
}
