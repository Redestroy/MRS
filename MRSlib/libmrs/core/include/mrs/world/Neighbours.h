#pragma once
// The robots around this one (spec 15 §3.3), from relative position measurements or peer states,
// and the repulsive force away from them (spec 15 §4).
#include <string>
#include <vector>

#include "mrs/world/Worldview.h"

namespace MRS {
	namespace Environment {
		struct Neighbour {
			std::string id;  // "r3"
			Enu rel;         // its position relative to this robot, ENU
			Enu rel_vel;     // its velocity relative to this robot; 0 when unknown
			bool measured = false;  // rel comes from a relative position sensor (rel.rN.enu)
		};

		// Every peer with a fresh relative position, by id. A fresh rel.rN.enu wins over
		// peer.rN.pose.enu − pose.enu; peer states count when at most peer_max_age old.
		std::vector<Neighbour> Neighbours(const Worldview& w, double t, double peer_max_age = 2.0);

		struct RepulsionConfig {
			double radius_xy = 4.0;  // m
			double radius_z = 2.0;   // m
			double gain = 0.5;       // m/s
			double max = 3.0;        // m/s, per neighbour
			double lookahead = 1.0;  // s
		};

		struct Repulsion {
			Enu force;            // m/s, ENU
			double nearest = -1;  // m to the nearest neighbour; negative with none
			int count = 0;        // neighbours inside the ellipsoid
		};

		// The repulsion of spec 15 §4.1 from these neighbours.
		Repulsion RepulsionFrom(const std::vector<Neighbour>& neighbours, const RepulsionConfig& c = {});
	}
}
