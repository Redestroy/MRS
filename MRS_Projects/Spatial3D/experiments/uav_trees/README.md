# WP7 generator examples

GeoJSON inputs for `mrs_generate` (spec 11 §4.3). Coordinates are around the reference of `platforms/webots/worlds/mission_3uav.mrs` (56.9496 N, 24.1052 E); altitudes are metres above it.

| File | What it holds |
|---|---|
| `field.geojson` | A field of about 115 × 100 m, covered at 20 m with a 20 m footprint and 20% overlap in 3 cells: the WP7 acceptance input (spec 11 §5) |
| `points.geojson` | One point per preset: picture, gimbal, LEDs, release package 7, land |
| `search.geojson` | Three points of interest searched in one tree: spirals of 25 m radius, 10 m footprint, 10% overlap |

Write a timeline and run it, for example on the Webots team world:

```
mrs_generate field.geojson ../../../../MRSlib/libmrs/platforms/webots/worlds/mission_3uav.mrs --land r1,r2,r3 > field.mrsl
```

`--land` ends each tree with every listed robot landing at home (spec 11 §3.6). The picture, gimbal and release presets need a robot with those devices (`spec/examples/mavic_delivery_webots.mrsd`); on the stock team world they are dumped as impossible.

In QuadSim with three UAVs and MRS-RTA (2026-10-08): `field.geojson` takes 125 s, all strips done by 82 s, then the landings; `search.geojson` takes 131 s, one spiral per robot.
