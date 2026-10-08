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

To fly another task set, copy it here from `MRS_Projects/Spatial3D/experiments/uav_spatial/` and
change the issuer's second argument.
