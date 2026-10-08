# WP6 results: a group of UAVs against one planned UAV

Plan §10, spec 10. Code at commit `c6893360` (`mrs_experiment`), QuadSim. 7,980 runs over 345 task sets, 10 seeds each, no stalled runs, no crashes, no geofence exits.

| File | What it is |
|---|---|
| `results.csv` | One row per run (spec 10 §5) |
| `results.csv.meta.txt` | Commit, mission, noise, battery, coefficients, planner settings (two runs: the main grid and the 15/60-task subset) |
| `tables.md` | Every table from `analyze.py results.csv` |

Grid: the 25 ported 2021 sets (15 tasks); generated sets of 30 tasks for 5 families × 4 dispatch types; and 15- and 60-task sets for random, cluster and multicluster with static and even dispatch. Conditions: S1, S1\*, and G-RTA, G-RTA-X and G-C at N = 2, 3, 5 and 8.

## Answer

**Yes: on every set family and dispatch type, a group finishes sooner than one planned UAV, and the speed-up stays below N.** Paired speed-up makespan(S1) / makespan(group), mean [95% CI]:

| | N = 2 | N = 3 | N = 5 | N = 8 |
|---|---|---|---|---|
| Ported 2021 sets, G-RTA | 1.62 [1.60, 1.63] | 1.99 [1.96, 2.01] | 2.32 [2.26, 2.37] | 2.44 [2.37, 2.51] |
| Generated, 30 tasks, G-RTA | 1.71 [1.70, 1.72] | 2.27 [2.25, 2.29] | 2.87 [2.82, 2.93] | 3.13 [3.05, 3.22] |
| Generated, 30 tasks, static dispatch, G-C | 1.75 [1.74, 1.76] | 2.39 [2.37, 2.42] | 3.39 [3.35, 3.43] | 4.42 [4.36, 4.48] |

What sets the speed-up:

1. **Dispatch timing.** With tasks released over time, the last release bounds every condition. Static dispatch (all tasks at once) gives the largest gains: 4.1–4.4 at N = 8, against 2.6–2.7 for even dispatch. The ported sets release 15 tasks over 70 s, so their speed-up flattens after N = 5.
2. **Set size.** More tasks per robot help larger teams: at N = 8, G-C goes from 3.22 (15 tasks) to 3.68 (60 tasks), on the same families and dispatch types.
3. **Spatial family.** A single tight cluster gains least (2.84 at N = 8, G-RTA) and an even grid most (3.45), as plan §10.1 expected. The family effect is smaller than the dispatch effect.

Per-robot efficiency falls with N: 0.83 at N = 2, 0.71 at N = 3, 0.52 at N = 5 and 0.36 at N = 8 (best condition, all sets).

## Allocators

The three group conditions are within a few percent of each other on makespan. Their confidence intervals overlap everywhere except:

* **G-C is best with static dispatch** (4.42 vs 4.07 at N = 8), where planning ahead pays.
* **G-C is worst on the ported sets** at N ≥ 3 (2.23 vs 2.44 at N = 8). It plans only the tasks it knows, and robots keep the task they are on (spec 10 §2.3).
* **G-C uses fewer robots and less flying**: at N = 8 it uses 6.2 robots on average, against 7.1 for G-RTA, and flies 1,278 m against 2,535 m. It also has the fewest separation breaches.
* **G-RTA-X** matches G-RTA on makespan, flies 22% less at N = 8, and sends 2.3× the messages (claims).

Robot-side decision time is about 20 µs for MRS-RTA and under 1 µs for the plan follower. The central planner's worst plan took 28 ms, and S1's worst replan 26 ms. Worst-case wall times are noisy because four runs shared the CPU.

## S1\* against S1

Knowing the whole timeline saves the single UAV 4.5% on the ported sets (S1\*/S1 = 0.955 [0.949, 0.961]) and 1% on the generated sets. One UAV is busy most of the time, so it rarely has slack to fly ahead. S1 is close to its own oracle bound, and the group speed-ups are not explained by a weak single-UAV baseline.

## Safety finding

Layers keep robots apart while they cruise, but not where they climb or descend. Separation breaches (closer than 1 m in 3D, both robots above 0.5 m) per run, at N = 8: G-RTA 13, G-RTA-X 3.8, G-C 0.05. Traces show two causes:

* Under open MRS-RTA, two robots fly to the same task and meet at its target.
* In all conditions, an idle robot hovers where its last task ended, and another robot climbs or descends through that point to a nearby task.

QuadSim has no collision model, so these runs did not crash. In Webots or real flight, they would. This is the reactive avoidance of WP12. A cheaper first step is for idle robots to climb back to their own layer. That step would remove the second cause and is not yet implemented.

## Limits

* QuadSim, not Webots (spec 10 §1). The Webots team world exists, but no Webots batch has been run yet.
* No battery swaps (200 Wh battery). No G-CBBA or G-STA yet.
* The simulated radio is lossless and passes each message through unchanged.
