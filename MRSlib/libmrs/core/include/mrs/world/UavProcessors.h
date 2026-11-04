#pragma once
// The version 0.1 UAV processors (spec 05 §5.2, §5.3) and the worldview set-up around them.
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "mrs/protocol/Record.h"
#include "mrs/world/Neighbours.h"
#include "mrs/world/Processor.h"

namespace MRS {
	namespace Environment {
		// The package flight of the sling (spec 18 §3.3).
		struct DeliveryConfig {
			double dest_tol = 1.0;         // m: payload.at_dest within this horizontal distance
			double via_offset = 8.0;       // m: the largest sideways offset of the path's via point
			double via_radius = 4.0;       // m: the via point counts as passed this close
			double approach = 2.5;         // m: the package's bottom above the destination on arrival
			double grounded_ratio = 0.3;   // payload.grounded when the load is below this share of the mass
		};

		struct UavWorldviewConfig {
			double alpha = 0.85;  // KinematicsEstimator position gain
			double beta = 0.3;    // velocity gain
			double gamma = 0.3;   // acceleration low-pass
			double battery_low = 0.3;
			double battery_critical = 0.15;
			double airborne_agl = 0.3;
			double landed_agl = 0.15;
			double landed_speed = 0.1;
			double landed_time = 1.0;
			double home_radius = 1.0;
			double detection_radius = 2.0;
			RepulsionConfig repulsion;  // spec 15 §4
			DeliveryConfig delivery;    // spec 18 §3.3
		};

		// Every UAV processor, in catalog order.
		std::vector<std::unique_ptr<IViewProcessor>> MakeUavProcessors(const UavWorldviewConfig& config = {});
		// The default source orders of spec 05 §5.1.
		void SetUavSourceOrders(Worldview& w);

		// Writes the mission fields of a mission header (H_M, spec 06 §6) for robot `robot_id`: the geo
		// reference, home.enu, layer.alt and geofence.*, stamped t.
		void ApplyMissionHeader(const Protocol::Record& header, std::int64_t robot_id, Worldview& w, double t);

		// Worldview, chain and clock of one robot.
		class WorldModel {
		public:
			// The UAV chain, pruned to what a robot producing `views` can use.
			static WorldModel ForViews(const std::set<std::string>& views, const UavWorldviewConfig& config = {});

			// One update at mission time t; views come from the sensor block.
			void Update(const std::vector<View>& views, double t);
			double Dt() const { return dt_; }

			Worldview world;
			ProcessorChain chain;

		private:
			double last_t_ = -1.0;
			double dt_ = 0.0;
		};

		// Heading (0 = North, clockwise) from ENU yaw (0 = East, counter-clockwise), in [0, 2π) (spec 00 §3).
		double HeadingFromYaw(double yaw);
	}
}
