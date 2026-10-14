# 10 Evaluation harness: planner baseline, task sets and the batch runner

Status: **WP6**, spec version `0.1`. Plan reference: §10 (evaluation), §11 (WP6). Builds on specs [06](06_Messages.md) (messages, timeline), [08](08_Robot_and_Flight.md) (flight, QuadSim) and [09](09_MRS_Layer.md) (MRS layer, MRS-RTA, issuer).

## 1. Scope

WP6 answers the question of plan §10.1 for the conditions that exist so far (S1, S1*, G-RTA, G-RTA-X, G-C):

* the **route planner** and the planner allocators: S1 and S1* on one UAV, and the central planner of G-C (§2);
* **task sets**: the ported 2021 sets, generators for the new 3D sets, and the oracle form (§3);
* the **team harness** and one **evaluation run** with its metrics (§4, §5);
* the **batch runner** `mrs_experiment` (§6) and the **analysis script** (§7).

Code: `MRS::Algorithms` (`algorithms/Planner.h`) for the planner; `MRS::Sim` in the new `sim/` library (`mrs_sim`) for QuadSim, the team harness, generators and runs; `tools/mrs_experiment.cpp`.

Runs fly in QuadSim (spec 08 §9), not Webots: the grid has thousands of runs, and QuadSim flies a run in about a second. The Webots team world (`platforms/webots/worlds/mavic_team.wbt`) checks the same controllers on the real simulator; a Webots batch runner is left for later (§8).

## 2. Planner

### 2.1 Task geometry and the travel model

`GeometryOf(task)` reads what the planner needs from a task record: the target (the first `C_P3` in the tree), the release (the latest `C_T`, in seconds; 0 without one) and the hold (the sum of its `A_W` and `A_HD` durations).

`TravelModel::Time(a, b, layer)` estimates a leg:

* a leg shorter than `switch_radius` across, or any leg in `DIRECT` mode: `max(across / v_xy, |Δz| / v_z)`;
* `LAYERED`: climb or descend to the robot's layer, fly across, then to the target altitude;
* `ALTITUDE_FIRST`: the same through `max(cruise_alt, target z)`, as `flyto.altitude_first` flies.

Defaults, measured on QuadSim with the spec 08 gains: `v_xy` 7.5 m/s (mean over a leg), `v_z` 2.0 m/s, `settle` 1.5 s per task, `takeoff` 0, `switch_radius` 3 m, `cruise_alt` 20 m. Measured legs at 15 m with a 20 m layer: 10 m in 7.2 s, 20 m in 9.0 s, 40 m in 11.8 s, 80 m in 16.7 s; ground to 15 m in 7.9 s. The model's predictions in a G-C run matched the completions within about 1 s.

### 2.2 Route planning

`PlanRoutes(robots, tasks, now, config)` returns one route per robot, the planned completion of every task and the planned makespan from `now`:

1. **Insertion.** Tasks in release order go, one at a time, to the robot and route position that cost least.
2. **Improvement.** Relocate (move a task to any position on any route), swap (exchange two tasks) and 2-opt (reverse a stretch of one route), each applied when it lowers the cost, for up to `improve_rounds` (30) passes or until none helps.

A task does not start before its release. A robot listed in a task's `excluded` set never gets it; a task every robot excludes is returned in `unassigned`. A robot's `fixed` task stays first on its route. Equal inputs give equal plans.

The cost is lexicographic, set by `PlannerConfig::objective`:

* `MAKESPAN`: the last planned completion, then the sum of completions;
* `COMPLETION`: the sum of completions, then the makespan.

### 2.3 Allocators

* **`PlannerAllocator` (S1, S1\*).** One UAV plans one route over every eligible task in its pool, on the first tick after a task arrives, with itself as the only robot (`fixed` = the task it has started). It does the first eligible task of the route. Objective `MAKESPAN`, mode `ALTITUDE_FIRST`. With the oracle timeline (§3.3) it knows every task from the start: that is S1\*.
* **`CentralPlanner` and `PlanFollowerAllocator` (G-C).** The issuer runs the planner (`TaskIssuer::SetPlanner`). The planner follows the robots through `M_STATE` (pose, current task) and `M_DUMP` (a static dump excludes that robot from the task); robots and layers come from the mission header. The issuer replans every open task when a task is dispatched, a task is done or failed, or a static dump arrives, and sends each robot its route in an `M_PLAN` (spec 06 §4) with a new revision. The robot does the first task of its latest route that it holds. Objective `COMPLETION`, mode `LAYERED`.

Commitment (found in WP6): a robot keeps the task it is on (`fixed` = its current task from `M_STATE`). Without that, each replan moved tasks between robots in flight; on task set 1 one task waited 86 s while robots turned back and forth.

`COMPLETION` is used for G-C because tasks keep arriving: a makespan objective over the tasks known so far let the newest task decide the plan and left early tasks waiting. On five ported sets it was equal or better than `MAKESPAN` (makespan 109 vs 110 s, 106 vs 115 s, equal on three).

## 3. Task sets

### 3.1 Ported 2021 sets

`experiments/uav_spatial/tasksets_2021/taskset1…25.mrsl` (spec 09 §7): 15 tasks each, every target at 15 m, dispatched about every 5 s.

### 3.2 Generated sets

`GenerateTaskSet(GenConfig)` writes a timeline of LED tasks like the ported ones (fly to a 3D point, LED on 3 s, off 3 s). The generator uses SplitMix64, so a seed gives the same file on every platform.

| Family | Targets in ±`half_size` (60 m) |
|---|---|
| `grid` | an even grid, visited in a shuffled order |
| `radial` | 8 spokes from the origin, radius 0.2–1.0 × `half_size` |
| `cluster` | one Gaussian cluster (σ 8 m) |
| `multicluster` | four Gaussian clusters (σ 6 m), tasks in turn |
| `random` | uniform |

| Dispatch | Release times |
|---|---|
| `static` | every task at `first` (1 s) |
| `even` | every `interval` (5 s) |
| `clustered` | bursts of `burst` (5) tasks, `burst_gap` (25 s) apart |
| `random` | exponential gaps with mean `interval` |

Altitudes are uniform in 5–25 m, positions rounded to 1 cm. Ids are `op.1…`.

### 3.3 Oracle form

`OracleTimeline(timeline)` sends every task at time 0. Each task becomes a `T_S` with the same id and priority, with two children:

1. a `T_A` whose start condition is the task's `C_P3` and whose action is `A_N` (fly there);
2. a `T_A` that starts on `C_T` = the original release in milliseconds, with the task's end condition and actions.

So a robot that knows the future may fly to a task before its release and wait there, but cannot do it earlier. Makespan and latency are always measured from the original releases (`ReleaseTimes`).

Since WP7 a team splits tree tasks between robots (spec 11 §2). The single-UAV conditions turn that off (`MrsConfig::split_trees`, spec 11 §2.6), so the robot runs each oracle `T_S` itself as above.

### 3.4 Seeds

A seed sets the home order (the row of homes is shuffled with SplitMix64), the GPS noise sequence (0.05 m) and, for generated sets, the set itself.

## 4. Team harness

`Sim::Team(files, TeamConfig)` builds N QuadSim UAVs from the robot definition and port map, each with its own MRS layer and allocator, on one broadcast medium (`Air`) with the issuer. A robot built `without_leds` has its LEDs removed from both files. `Run(seconds, stop_when_done)` ticks the issuer and the robots, delivers the messages, steps QuadSim, and watches:

* separation: a breach is a pair, both above 0.5 m, coming closer than `separation` (1 m) in 3D; the minimum is kept;
* geofence exits; crashes.

`TimedAllocator` wraps each allocator and measures every `Select` call (wall clock).

The simulated radio delivers each message as the sender wrote it. It would not have caught the trailing NUL the Webots issuer sent (fixed on the Webots side, PR #3).

## 5. One run

`RunOne(files, RunSpec)` flies one set under one condition and seed:

* mission: fence ±100 m, 60 m high, layers from 20 m every 3 m, homes a row 4 m apart at y = −5;
* battery 200 Wh (no swaps; plan §14 question 2 stays open), time limit 4000 s;
* S1 and S1\* fly `flyto.altitude_first` (behaviour priority 11); groups fly the layered default.

| Metric | Meaning |
|---|---|
| `completed` | every task done or failed before the time limit |
| `makespan_s` | first release to last completion; a stalled run counts to the limit |
| `latency_mean_s`, `latency_max_s` | release to completion |
| `distance_m`, `energy_wh` | summed over robots |
| `duplicates` | `M_DONE` messages beyond the first, summed over tasks |
| `busy_robots` | robots that completed at least one task |
| `messages`, `bytes` | on the shared medium |
| `decision_mean_us`, `decision_worst_us` | robot-side `Select`, wall clock |
| `plan_calls`, `plan_worst_ms` | central replans (G-C) or own replans (S1, S1\*) |
| `separation_breaches`, `min_separation_m`, `fence_exits`, `crashed` | safety |

Wall-clock times are measured with four runs in parallel, so their worst cases include scheduling noise.

## 6. Batch runner

```
mrs_experiment run [--ported DIR] [--gen FAMILY:DISPATCH:TASKS]... [--seeds A-B]
                   [--cond S1|S1*|G-RTA:N,..|G-RTA-X:N,..|G-C:N,..]... [--jobs J]
                   [--out results.csv] [--limit SECONDS] [--sets-out DIR] [--commit HASH]
mrs_experiment gen FAMILY DISPATCH TASKS SEED      print a generated set
mrs_experiment oracle FILE                         print the oracle form
```

Every set and seed runs under every condition and N. Each run appends one CSV row; rows already in the file (by set, seed, condition, N) are skipped, so an interrupted grid resumes. `<out>.meta.txt` records the commit, world, mission, noise, battery, limit, RTA and size coefficients, travel model and planner settings. `--sets-out` writes the generated sets.

## 7. Analysis

`experiments/uav_spatial/analyze.py results.csv [--out results.md]` (Python standard library only) prints Markdown tables:

* makespan per condition and source (ported, generated), mean with a 95% bootstrap interval;
* paired speed-up `makespan(S1) / makespan(condition)` per (set, seed), by source, by family and by dispatch (at the main task count), and by task count;
* S1\*/S1;
* the secondary metrics per condition;
* the best condition per N.

Intervals are percentile bootstraps (2000 resamples, fixed RNG seed), so the tables are reproducible.

## 8. Not in WP6

* G-CBBA and G-STA (WP8 onwards); tree-task sets (tree tasks themselves are WP7, spec 11).
* A Webots batch runner and per-N world generator: the grid runs in QuadSim (§1).
* Battery swaps during a run.
