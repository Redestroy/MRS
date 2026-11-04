// Zones and queues (spec 18 §4): robots that want the same place line up by the stamp of their
// want, then by robot number; the first `capacity` hold it. The layer writes zone.<id>.mine,
// zone.<id>.place and zone.<id>.hold into the robot's worldview, and zone.dest.* for the
// destination area on the tag of the carried package.
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "mrs/algorithms/MrsLayer.h"

namespace MRS {
	namespace Algorithms {
		void MrsLayer::OnZoneWant(std::int64_t zone, double t) {
			auto it = zones_.find(zone);
			if (it == zones_.end()) return;  // not a zone of the mission: never queued
			ZoneState& z = it->second;
			if (z.wanted) return;
			z.wanted = true;
			z.entered = false;
			z.stamp = t;
			SendZone(zone, "want", t, t);
		}

		void MrsLayer::OnZoneMessage(const Comm::Message& m, double t) {
			if (m.SlotCount() < 3) return;
			auto it = zones_.find(m.Slot(0).i);
			if (it == zones_.end()) return;
			if (m.Slot(1).s == "want") it->second.wants[m.sender] = {m.Slot(2).n, t};
			else it->second.wants.erase(m.sender);
		}

		void MrsLayer::SendZone(std::int64_t zone, const char* op, double stamp, double t) {
			auto r = messenger_.Begin("M_ZONE", "all", t);
			r.fields.push_back(Protocol::Field::MakeInt(zone));
			r.fields.push_back(Protocol::Field::MakeText(Protocol::FieldType::Id, op));
			r.fields.push_back(Protocol::Field::MakeNum(stamp));
			messenger_.Post(r);
		}

		void MrsLayer::UpdateZones(double t) {
			if (zones_.empty()) return;
			auto& w = robot_.World();
			const auto px = w.Scalar("pose.enu.x", t), py = w.Scalar("pose.enu.y", t);
			for (auto& [id, z] : zones_) {
				for (auto it = z.wants.begin(); it != z.wants.end();)
					it = t - it->second.second > c_.zone_timeout ? z.wants.erase(it) : std::next(it);
				int place = -1;
				if (z.wanted) {
					// Release: the robot has no unit any more, or it was in the zone and has left it.
					const double d = px && py ? std::hypot(*px - z.spec.x, *py - z.spec.y) : 1e9;
					if (assigned_.empty() || (z.entered && d > z.spec.radius + 1.0)) {
						z.wanted = false;
						SendZone(id, "free", z.stamp, t);
					} else {
						if (t - z.sent >= c_.zone_renew) {
							SendZone(id, "want", z.stamp, t);
							z.sent = t;
						}
						place = 0;
						for (const auto& [robot, want] : z.wants)
							if (want.first < z.stamp || (want.first == z.stamp && RobotNumber(robot) < RobotNumber(self_))) ++place;
						if (place < z.spec.capacity && d <= z.spec.radius) z.entered = true;
					}
				}
				const std::string p = "zone." + std::to_string(id);
				const bool mine = z.wanted && place >= 0 && place < z.spec.capacity;
				w.SetBool(p + ".mine", mine, t, "zones");
				w.SetScalar(p + ".place", place, t, "zones");
				const double line = std::max(0, place - z.spec.capacity + 1);
				w.SetVec3(p + ".hold", z.spec.hold_x, z.spec.hold_y, c_.hold_base + line * c_.hold_step, t, "zones");
			}
			// The destination zone of the carried package.
			const auto area = w.Scalar("payload.area", t);
			const auto held = w.Bool("payload.held", t);
			if (area && held && *held) {
				const std::int64_t id = c_.dest_zone_base + static_cast<std::int64_t>(std::llround(*area));
				w.SetScalar("zone.dest.id", static_cast<double>(id), t, "zones");
				if (!zones_.count(id)) w.SetBool("zone.dest.mine", true, t, "zones");  // no zone there: nothing to wait for
				else w.SetBool("zone.dest.mine", *w.Bool("zone." + std::to_string(id) + ".mine", t), t, "zones");
			} else {
				w.Invalidate("zone.dest.id", t);
				w.SetBool("zone.dest.mine", false, t, "zones");
			}
		}
	}
}
