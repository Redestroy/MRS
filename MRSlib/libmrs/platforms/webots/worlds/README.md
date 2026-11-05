# Webots worlds

## mavic_team.wbt (WP5 team, spec 09 §8)

Three Mavic 2 Pro (`r1`, `r2`, `r3`) run `mrs_uav` with open MRS-RTA, and a supervisor Robot
`issuer` runs `mrs_issuer` with the 2021 task set 1 (`taskset1.mrsl`, 15 tasks over 71 s).

1. Build libmrs with `WEBOTS_HOME` set. The `mrs_uav` and `mrs_issuer` binaries and the robot files
   (`mavic_webots.mrsd`, `.mrsp`, `uav_behaviours.mrsb`) land in `../controllers/<name>/`.
2. Open `mavic_team.wbt` in Webots (R2025a). Webots finds the controllers in `../controllers`, so
   open the world from this folder, not from a copy.
3. Run. The robots wait on the ground for the mission header, then take off as tasks arrive. Each
   console line `t rN task op.K` is an assignment. When every task has ended, the issuer prints the
   makespan, writes `mavic_team_results.csv` here and pauses the simulation.

What the world sets, and what any other team world needs:

* `WorldInfo`: `gpsCoordinateSystem "WGS84"` and `gpsReference 56.9496 24.1052 10`, the reference in
  the mission header (`mission_3uav.mrs`).
  Also `defaultDamping` with linear and angular 0.5, as in Webots' own Mavic world; the FCU gains in
  `mavic_webots.mrsd` (`kp_att 20`) are tuned for it, and without it the Mavics lose stability.
* Each Mavic: `controllerArgs` = its robot id (`"1"`, `"2"`, …; add `"rta-x"` as a second argument for
  exclusive MRS-RTA), `battery [180000, 180000, 0]` (50 Wh in joules; an empty battery field gives
  no battery sensor and the robot reports a fault), and an `Emitter "emitter"` and a
  `Receiver "receiver"` on channel 1 in `bodySlot`.
* The homes in the mission header match the Mavic translations (x −4, 0, 4 at y −5). For more
  robots, add Mavics and extend the header's robot list (`n`, then `id x y z` per robot).
* The issuer's `controllerArgs`: mission header, timeline and results paths, relative to
  `../controllers/mrs_issuer/` (the controller's working folder).

The issuer also writes `mavic_team_track.csv` here: every robot's position and attitude every
0.1 s, with the mission header and the task events (spec 17 §2). Replay it outside Webots with the
MRS dot viewer (build with `-DMRS_BUILD_VIEWER=ON`):

```
mrs_dot_viewer mavic_team_track.csv taskset1.mrsl
```

Add `"--track-period" "0.05"` to the issuer's `controllerArgs` for denser rows, or `"--track" "none"`
to turn it off.

To fly another task set, copy it here from `MRS_Projects/Spatial3D/experiments/uav_spatial/` and
change the issuer's second argument.

## mavic_delivery.wbt (package delivery, spec 18 §9)

Three Mavic 2 Pro with a sling deliver 12 packages from three piles (x = −6, 0, 6 on y = 0) to four
destinations (`delivery_destinations.csv`). r3 starts at 40 % battery and charges at the pad
(x 10, y −8) first. The world and its files are generated:

```
mrs_generate delivery 12 delivery_sources.csv delivery_destinations.csv --zones --at 2 \
    --manifest delivery_manifest.csv --zones-out delivery_zones.csv > delivery_12.mrsl
python3 make_delivery_world.py delivery_manifest.csv delivery_sources.csv delivery_zones.csv \
    delivery_12.mrsl mavic_delivery.wbt --low 3:0.4
```

The script writes `mavic_delivery.wbt` and the mission header `mavic_delivery_mission.mrs`. Run it as
the team world: build with `WEBOTS_HOME` set (the controller folder gets `mavic_sling_webots.mrsd`
and `.mrsp`), open the world from this folder, run. When every delivery has ended the issuer writes
`mavic_delivery_results.csv` and the track, and pauses.

What the world sets:

* Each package is a Solid box (0.3 × 0.3 × 0.25 m, its mass from the manifest) whose `description`
  is its tag, `tag <id> <dest x> <dest y> <dest z> <area> <mass>`, with a passive `Connector`
  "loop" (model `sling`) 0.1 m above its top.
* Each Mavic has `supervisor TRUE` (the tag reader looks up the package on the hook) and the sling
  in `bodySlot`: a 2 m line on a `BallJoint` below the centre of mass, a `SliderJoint` spring with a
  `PositionSensor` "load cell" (150 N/m), and the active `Connector` "hook". Its `controllerArgs`
  name the sling robot files and pass `--zones delivery_zones.csv --pad 101,10,-8,16,-8`.
* A `Charger` at the pad (radius 1.2 m) charges a landed Mavic's battery.

Set `MRS_SLING_DEBUG=1` in the environment Webots starts from to print the hook's state once a
second and each catch.

Status (2026-11-05, headless Webots R2025a on Linux): the hook catches top packages, Mavics fly them
on the line, lower and release them at their destinations, and r3 charges at the pad (about 45 s).
One full run delivered 4 of the 12 packages, then stalled. The open problem is the swing of the
loaded line: descending 15 to 25 m from the cruise layer to the drop height, a Mavic can overshoot.
The package then hits the ground, the line jerks, and the Mavic flips. More damping on the line's
ball joint (1.0) made it worse (all three flipped). The next steps are a slower loaded descent and
FCU gains for loaded flight.
