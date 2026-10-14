// WP3 acceptance check in Webots: the logged pose.enu, alt.agl, heading and vel.enu are compared
// with supervisor ground truth (plan §11, WP3).
//
// The robot is built from its .mrsd through the device layer, its views go through the worldview
// pipeline, and the motors are driven through the actuator block. The flight itself uses the simple
// stabiliser of the Cyberbotics Mavic example (no position control; that is WP4): climb to 5 m,
// then fly forward while turning, so that every field changes.
//
// Usage (controllerArgs): [definition.mrsd] [log.csv]. Defaults: mavic_webots.mrsd next to the
// controller and mrs_worldview_check.csv. The Mavic node needs supervisor TRUE, and WorldInfo needs
// gpsCoordinateSystem "WGS84".
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

#include <webots/Field.hpp>
#include <webots/Node.hpp>
#include <webots/Supervisor.hpp>

#include "mrs/device/Robot.h"
#include "mrs/device/uav/UavDevices.h"
#include "mrs/platform/WebotsPlatform.h"
#include "mrs/protocol/Parser.h"
#include "mrs/world/UavProcessors.h"

using namespace MRS;

namespace {
	constexpr double kPi = 3.14159265358979323846;

	// Tolerances of the check.
	constexpr double kPoseTol = 0.5;     // m
	constexpr double kAltTol = 0.5;      // m
	constexpr double kHeadingTol = 0.05; // rad
	constexpr double kVelTol = 0.3;      // m/s

	std::string ReadFile(const std::string& path) {
		std::ifstream in(path, std::ios::binary);
		std::stringstream s;
		s << in.rdbuf();
		return s.str();
	}

	double AngleDiff(double a, double b) {
		double d = std::fmod(a - b, 2 * kPi);
		if (d > kPi) d -= 2 * kPi;
		if (d < -kPi) d += 2 * kPi;
		return std::fabs(d);
	}

	const double* GpsReference(webots::Supervisor& sup) {
		webots::Field* children = sup.getRoot()->getField("children");
		for (int k = 0; k < children->getCount(); ++k) {
			webots::Node* n = children->getMFNode(k);
			if (n->getTypeName() == "WorldInfo") return n->getField("gpsReference")->getSFVec3f();
		}
		return nullptr;
	}

	Device::ActionMap Motors(const double v[4]) {
		Device::ActionMap map;
		map.combined = true;
		const char* names[4] = {"m_fl", "m_fr", "m_rl", "m_rr"};
		for (int k = 0; k < 4; ++k)
			map.entries.push_back({names[k], {"A_MOT", *Device::PackArgument(Device::ArgLayout::F64, std::vector<double>{v[k]})}});
		return map;
	}
}

int main(int argc, char** argv) {
	webots::Supervisor supervisor;
	const std::string definition = argc > 1 ? argv[1] : "mavic_webots.mrsd";
	const std::string log_path = argc > 2 ? argv[2] : "mrs_worldview_check.csv";

	Platform::WebotsPlatform platform(supervisor);
	Device::DeviceRegistry registry;
	Device::Uav::RegisterUavDevices(registry);
	Device::Robot robot;
	try {
		robot = Device::BuildRobot(ReadFile(definition), registry, platform);
	} catch (const std::exception& e) {
		std::fprintf(stderr, "build failed: %s\n", e.what());
		return 1;
	}
	for (const auto& f : robot.faults) std::printf("FAULT %s: %s\n", f.node.c_str(), f.reason.c_str());

	auto model = Environment::WorldModel::ForViews(robot.self.views);
	const double* ref = GpsReference(supervisor);
	if (!ref) {
		std::fprintf(stderr, "no WorldInfo.gpsReference\n");
		return 1;
	}
	char header[256];
	std::snprintf(header, sizeof header, "H: H_M check %.10f %.10f %.4f -500 500 -500 500 120 20 5 120 1 %lld 0 0 0/", ref[0],
	              ref[1], ref[2], static_cast<long long>(robot.self.id));
	auto parsed = Protocol::Parse(header);
	Environment::ApplyMissionHeader(parsed.document.records.at(0), robot.self.id, model.world, 0.0);

	std::ofstream log(log_path);
	log << "t,source_pose,x,y,z,tx,ty,tz,agl,tagl,heading,theading,vx,vy,vz,tvx,tvy,tvz\n";
	webots::Node* self = supervisor.getSelf();
	double max_pose = 0, max_alt = 0, max_heading = 0, max_vel = 0;
	const double kVerticalThrust = 68.5, kVerticalOffset = 0.6, kVerticalP = 3.0, kRollP = 50.0, kPitchP = 30.0;

	while (platform.Step()) {
		const double t = platform.Time();
		model.Update(robot.blocks.sensors.Sample(t), t);
		const auto& w = model.world;

		// Stabiliser of the Cyberbotics Mavic example, fed from the worldview.
		const double roll = w.Scalar("att.roll", t).value_or(0), pitch = w.Scalar("att.pitch", t).value_or(0);
		const double p = w.Scalar("rate.body.x", t).value_or(0), q = w.Scalar("rate.body.y", t).value_or(0);
		const double altitude = w.Scalar("alt.agl", t).value_or(0);
		const double target = 5.0;
		const double pitch_disturbance = t > 10 ? -1.0 : 0.0, yaw_disturbance = t > 10 ? 0.5 : 0.0;
		const double roll_in = kRollP * std::clamp(roll, -1.0, 1.0) + p;
		const double pitch_in = kPitchP * std::clamp(pitch, -1.0, 1.0) + q + pitch_disturbance;
		const double dz = std::clamp(target - altitude + kVerticalOffset, -1.0, 1.0);
		const double vertical = kVerticalP * dz * dz * dz;
		const double v[4] = {kVerticalThrust + vertical - roll_in + pitch_in - yaw_disturbance,
		                     -(kVerticalThrust + vertical + roll_in + pitch_in + yaw_disturbance),
		                     -(kVerticalThrust + vertical - roll_in - pitch_in + yaw_disturbance),
		                     kVerticalThrust + vertical + roll_in - pitch_in - yaw_disturbance};
		robot.blocks.actuators.Dispatch(Motors(v), t);
		robot.blocks.actuators.Update(t);

		// Ground truth.
		const double* pos = self->getPosition();
		const double* rot = self->getOrientation();  // row-major 3x3
		const double* vel = self->getVelocity();
		const double truth_heading = Environment::HeadingFromYaw(std::atan2(rot[3], rot[0]));
		const auto x = w.Scalar("pose.enu.x", t), y = w.Scalar("pose.enu.y", t), z = w.Scalar("pose.enu.z", t);
		const auto agl = w.Scalar("alt.agl", t), heading = w.Scalar("heading", t);
		const auto vx = w.Scalar("vel.enu.x", t), vy = w.Scalar("vel.enu.y", t), vz = w.Scalar("vel.enu.z", t);
		if (!x || !agl || !heading || !vx) continue;
		log << t << ',' << w.SelectedSource("pose.enu") << ',' << *x << ',' << *y << ',' << *z << ',' << pos[0] << ',' << pos[1]
		    << ',' << pos[2] << ',' << *agl << ',' << pos[2] << ',' << *heading << ',' << truth_heading << ',' << *vx << ',' << *vy
		    << ',' << *vz << ',' << vel[0] << ',' << vel[1] << ',' << vel[2] << '\n';
		if (t < 2.0) continue;
		max_pose = std::max({max_pose, std::fabs(*x - pos[0]), std::fabs(*y - pos[1]), std::fabs(*z - pos[2])});
		max_alt = std::max(max_alt, std::fabs(*agl - pos[2]));
		max_heading = std::max(max_heading, AngleDiff(*heading, truth_heading));
		max_vel = std::max({max_vel, std::fabs(*vx - vel[0]), std::fabs(*vy - vel[1]), std::fabs(*vz - vel[2])});
		if (t >= 40.0) break;
	}

	const bool pass = max_pose <= kPoseTol && max_alt <= kAltTol && max_heading <= kHeadingTol && max_vel <= kVelTol;
	std::printf("max |pose.enu - truth| %.3f m, |alt.agl - truth| %.3f m, |heading - truth| %.4f rad, |vel.enu - truth| %.3f m/s: %s\n",
	            max_pose, max_alt, max_heading, max_vel, pass ? "PASS" : "FAIL");
	return pass ? 0 : 2;
}
