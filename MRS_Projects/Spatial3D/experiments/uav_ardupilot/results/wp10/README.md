# WP10 results: task set 1 in ArduCopter SITL (spec 14 §9)

ArduCopter 4.6.3 SITL (`--model +`, real time), `mission_5uav.mrs`, the 2021 task set 1 port
(`../uav_spatial/tasksets_2021/taskset1.mrsl`, 15 tasks), robots at the mission header's homes.
Flown on 2026-10-09 with `run_sitl.sh` and summarized with `summarize.py`.

* `ts1_n<N>_<alloc>/`: the runs with the committed `run_sitl.sh` and `sitl.parm` (20 Ah pack).
* `repeat1/`: an earlier run of the N = 3 conditions with SITL's default 3.3 Ah pack (no robot got
  near the low-battery rule at N = 3), used as a repeat.
* `battery_3300mAh/ts1_n1_rta/`: one copter on the default pack. It reached the low-battery rule
  (spec 08 §4) after 14 tasks at t ≈ 357 s, flew home, landed for a swap and waited, as specified;
  the operator was stopped. That is why `sitl.parm` exists.
* `single_n1_rta/`: `single_flight.mrsl` (take off to 6 m, fly 8 m, flash LEDs, land): 3/3 tasks.
* `quadsim_taskset1.csv`: the QuadSim runs of the same set (mrs_experiment, seed 1) for comparison.

Every SITL run completed all its tasks, with no faults, failsafes or duplicate completions.

## Makespan, SITL against QuadSim and Webots

| condition | SITL run 1 (s) | SITL run 2 (s) | QuadSim (s) | SITL / QuadSim | Webots Mavic (s) |
|---|---|---|---|---|---|
| S1 (1 copter, RTA) | 319.8 | | 276.6 | 1.16 | |
| G-RTA, N = 3 | 147.3 | 147.7 | 134.0 | 1.10 | 130.7 |
| G-RTA-X, N = 3 | 161.4 | 153.9 | 136.1 | 1.13–1.19 | |
| G-CBBA, N = 3 | 161.2 | 160.5 | 117.9 | 1.36–1.37 | |
| G-LDTA2, N = 3 | 156.1 | 161.3 | 136.1 | 1.15–1.18 | |

Speed-up of the team of 3 over one copter: 2.2 with RTA in SITL (2.1 in QuadSim).

Mean task latency (dispatch to completion) is 52–60 s in SITL at N = 3, against 35–37 s in QuadSim.

## Reading

* ArduCopter flies the same tasks 10–20% slower than QuadSim (S1: 16%). This is inferred, not measured
  per leg: ArduCopter's position controller limits acceleration and jerk (WPNAV_ACCEL, S-curves) and
  climbs at its own limit, so the 10–40 m legs of task set 1 rarely reach the 8 m/s cruise speed, while
  QuadSim's travel model flies at 7.5 m/s with a fixed 1.5 s settle. The Webots Mavic (130.7 s) is
  close to QuadSim.
* Run-to-run spread is up to 5% (RTA-X 153.9 against 161.4 s). SITL runs in real time over UDP, so
  message timing and the allocators' tie-breaks differ between runs; one run per condition is not
  enough to rank the allocators in SITL.
* CBBA loses most (117.9 s in QuadSim, about 161 s in SITL). Its bundles are planned from the travel
  estimate, which matches QuadSim's dynamics; with slower real legs the planned sequences are no longer
  the best, while RTA re-decides every tick (inferred). Calibrating the size estimate's travel model on
  the platform (as the energy model already is, spec 08 §6) would test this.
* The battery: SITL's default pack drains about 7× faster than the 50 Wh Mavic model, which the
  energy model's calibration followed; the low-battery path worked end to end on ArduPilot.
