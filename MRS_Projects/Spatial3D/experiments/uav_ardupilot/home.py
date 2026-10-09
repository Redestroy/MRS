# home.py X Y Z: mission ENU point (m) -> ArduPilot SITL --home "lat,lon,alt,heading".
# The origin is the G_O of experiments/uav_spatial/mission_5uav.mrs.
import math, sys

LAT0, LON0, ALT0 = 56.9496, 24.1052, 10.0
R = 6378137.0
x, y, z = map(float, sys.argv[1:4])
lat = LAT0 + math.degrees(y / R)
lon = LON0 + math.degrees(x / (R * math.cos(math.radians(LAT0))))
print(f"{lat:.8f},{lon:.8f},{ALT0 + z:.2f},0")
