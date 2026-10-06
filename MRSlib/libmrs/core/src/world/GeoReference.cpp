#include "mrs/world/GeoReference.h"

#include <cmath>

namespace MRS {
	namespace Environment {
		namespace {
			constexpr double kPi = 3.14159265358979323846;
			constexpr double kA = 6378137.0;              // WGS-84 semi-major axis
			constexpr double kF = 1.0 / 298.257223563;    // flattening
			constexpr double kE2 = kF * (2.0 - kF);       // first eccentricity squared

			double Rad(double deg) { return deg * kPi / 180.0; }
			double Deg(double rad) { return rad * 180.0 / kPi; }

			void ToEcef(double lat, double lon, double alt, double& x, double& y, double& z) {
				const double s = std::sin(lat), c = std::cos(lat);
				const double n = kA / std::sqrt(1.0 - kE2 * s * s);
				x = (n + alt) * c * std::cos(lon);
				y = (n + alt) * c * std::sin(lon);
				z = (n * (1.0 - kE2) + alt) * s;
			}
		}

		GeoReference::GeoReference(double lat0_deg, double lon0_deg, double alt0_m)
		    : lat0_(Rad(lat0_deg)), lon0_(Rad(lon0_deg)), alt0_(alt0_m) {
			ToEcef(lat0_, lon0_, alt0_, x0_, y0_, z0_);
		}

		Enu GeoReference::ToEnu(double lat_deg, double lon_deg, double alt_m) const {
			double x, y, z;
			ToEcef(Rad(lat_deg), Rad(lon_deg), alt_m, x, y, z);
			const double dx = x - x0_, dy = y - y0_, dz = z - z0_;
			const double sl = std::sin(lat0_), cl = std::cos(lat0_), so = std::sin(lon0_), co = std::cos(lon0_);
			return {-so * dx + co * dy, -sl * co * dx - sl * so * dy + cl * dz, cl * co * dx + cl * so * dy + sl * dz};
		}

		void GeoReference::ToGeodetic(const Enu& e, double& lat_deg, double& lon_deg, double& alt_m) const {
			const double sl = std::sin(lat0_), cl = std::cos(lat0_), so = std::sin(lon0_), co = std::cos(lon0_);
			const double x = x0_ - so * e.x - sl * co * e.y + cl * co * e.z;
			const double y = y0_ + co * e.x - sl * so * e.y + cl * so * e.z;
			const double z = z0_ + cl * e.y + sl * e.z;
			// Bowring's iteration, a few steps are enough for mm accuracy near the surface.
			const double lon = std::atan2(y, x);
			const double p = std::sqrt(x * x + y * y);
			double lat = std::atan2(z, p * (1.0 - kE2));
			double alt = 0.0;
			for (int i = 0; i < 5; ++i) {
				const double s = std::sin(lat);
				const double n = kA / std::sqrt(1.0 - kE2 * s * s);
				alt = p / std::cos(lat) - n;
				lat = std::atan2(z, p * (1.0 - kE2 * n / (n + alt)));
			}
			lat_deg = Deg(lat);
			lon_deg = Deg(lon);
			alt_m = alt;
		}
	}
}
