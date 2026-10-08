# uav_spatial experiments

Inputs for the group-versus-single evaluation (plan §10, WP6). WP5 adds:

* `tasksets_2021/taskset1.mrsl` … `taskset25.mrsl`: the 25 E-puck task sets of 2021
  (`Simulations/MRS_Spacial_Tasks/Default_world/controllers/operator_robot/TaskSets`), ported with
  `mrs_port2021` (spec 09 §7): 0.1 m per 2021 unit, the 2021 point (500, 500) at the ENU origin,
  every target at 15 m, LED on for 3 s, off for 3 s. Dispatch times are the 2021 times in seconds.
* `mission_5uav.mrs`: a mission header for five UAVs that fits the ported sets.

In Webots: the supervisor runs `mrs_issuer mission_5uav.mrs tasksets_2021/taskset1.mrsl`, and each
Mavic runs `mrs_uav <id>` (spec 09 §8).

A ready three-UAV world with task set 1 is `MRSlib/libmrs/platforms/webots/worlds/mavic_team.wbt`; its
README lists what a team world needs (WGS84 GPS, battery, radio, homes).

WP6 adds `analyze.py` (spec 10 §7) and `results/wp6/`: the evaluation grid's CSV, its metadata, the tables and
the findings (`results/wp6/README.md`). Re-run with `mrs_experiment run` (spec 10 §6).
