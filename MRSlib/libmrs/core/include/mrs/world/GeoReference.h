#pragma once
// Geodetic WGS-84 to local ENU conversion around the mission's geo reference
// (spec 00 §3, spec 06 §6). Exact: geodetic -> ECEF -> ENU.
namespace MRS {
	namespace Environment {
		struct Enu {
			double x = 0.0, y = 0.0, z = 0.0;
		};

		class GeoReference {
		public:
			GeoReference() = default;
			GeoReference(double lat0_deg, double lon0_deg, double alt0_m);

			Enu ToEnu(double lat_deg, double lon_deg, double alt_m) const;
			void ToGeodetic(const Enu& enu, double& lat_deg, double& lon_deg, double& alt_m) const;

		private:
			double lat0_ = 0.0, lon0_ = 0.0, alt0_ = 0.0;  // radians, radians, metres
			double x0_ = 0.0, y0_ = 0.0, z0_ = 0.0;        // ECEF of the origin
		};
	}
}
