# Spatial3D: 3D spatial task allocation for UAV groups

This project builds a group of flight-capable UAVs on the MRS framework. The UAVs allocate and execute 3D spatial tasks. The project also answers one question: **does a group of N UAVs finish a set of spatial tasks faster than one UAV flying a planned mission?**

Branch: `spatial-3d`. Status: **WP8 done** (WP0–WP8). Library code is in [`MRSlib/libmrs`](../../MRSlib/libmrs); the WP6 findings are in [`experiments/uav_spatial/results/wp6`](experiments/uav_spatial/results/wp6/README.md); the WP7 generator examples are in [`experiments/uav_trees`](experiments/uav_trees/README.md); the WP8 tree-set results are in [`experiments/uav_spatial/results/wp8`](experiments/uav_spatial/results/wp8/README.md).

## Contents

| Path | What it is |
|---|---|
| [`docs/MRS_UAV_Implementation_Plan.md`](docs/MRS_UAV_Implementation_Plan.md) | The implementation plan: layers, objects, work packages WP0–WP12, the evaluation design, and JB's decisions (§15) |
| [`spec/`](spec/) | WP0 strict definitions. These are normative: code and task files must follow them |
| [`spec/examples/`](spec/examples/) | Example files written in the grammar. They will become the first round-trip test inputs |

## Specification documents

| # | Document | Defines |
|---|---|---|
| 00 | [Conventions](spec/00_Conventions.md) | Normative wording, units, frames, time, numbers, naming, versioning |
| 01 | [Protocol grammar](spec/01_Protocol_Grammar.md) | The text protocol: lexical rules, record structure, depth-first rule, references, EBNF |
| 02 | [Actions and functions](spec/02_Actions.md) | The action model, the 64-bit argument and its layouts, the action code registry, combined and parametric actions, functions |
| 03 | [Tasks, conditions and execution](spec/03_Tasks_and_Execution.md) | Task kinds and slots, states, conditions, requirements, tree links, the executor contract |
| 04 | [Devices and ports](spec/04_Devices_and_Ports.md) | Device-tree definition files, node kinds, capabilities, port requirements, the self model |
| 05 | [Worldview](spec/05_Worldview.md) | Field naming, types, units, staleness, the minimum UAV field set, views |
| 06 | [Messages](spec/06_Messages.md) | The message envelope, task ids, message types and payloads, the mission header, the task journal |
| 07 | [Build and test](spec/07_Build_and_Test.md) | Repository layout, CMake and compiler rules, test framework, the WP0 round-trip test suite |
| 08 | [Robot layer and flight](spec/08_Robot_and_Flight.md) | The flight control unit, the robot controller and its events, the safety supervisor, energy, journal and resume (WP4) |
| 09 | [MRS layer](spec/09_MRS_Layer.md) | Messages and transports, the task pool, allocation (MRS-RTA), the dump rule and the task issuer (WP5) |
| 10 | [Evaluation harness](spec/10_Evaluation.md) | The route planner (S1, S1*, G-C), task-set generators and the oracle form, the team harness, the batch runner and the analysis (WP6) |
| 11 | [Tree tasks and generators](spec/11_Trees_and_Generators.md) | Tree tasks split across robots, the point, path, coverage, perimeter and spiral search generators, GeoJSON input and `mrs_generate` (WP7) |
| 12 | [Splittable trees and MRS-STA](spec/12_STA.md) | Units (trees split only where they can be split), MRS-STA with the active-task stack, tree-task sets and evaluation v2 (WP8) |
| 13 | [CBBA, LDTA² and the communication budget](spec/13_CBBA_LDTA2.md) | CBBA, LDTA², the bitrate-limited channel and message pacing, evaluation v3 (WP9) |
| 14 | [The ArduPilot platform](spec/14_ArduPilot.md) | MAVLink ports, the GUIDED-mode flight control unit, pass-through processing, UDP between agents, the mock autopilot, SITL runs (WP10) |
| 15 | [Worldview sync, relative positions and repulsion](spec/15_Sync_and_Neighbours.md) | Sharing world fields and detections (M_SYNC, ask a peer), the ranging sensor, neighbours, the repulsion processor and its opt-in use, the sync behaviour and the capability guard (WP11) |

## WP0 acceptance (from the plan)

* Round-trip parse and write tests pass for every object kind defined in `spec/01`–`spec/06`.
* The skeleton builds on Windows and Linux.

## Where files live

* Project documents, the specification and experiments: `MRS_Projects/Spatial3D/` in the repository.
* Library code: `MRSlib/libmrs/` in the repository, next to the legacy `MRSlib/MRS/` (spec 07 §1).
* On JB's PC, the project folder is mirrored at `E:\Claude\MRS_Projects\Spatial3D\`.

## WP0 decisions (JB, 2026-10-05)

1. **Library location:** `MRSlib/`. The folder is `MRSlib/libmrs/` because `MRSlib/mrs` and `MRSlib/MRS` are the same folder on Windows.
2. **Expression functions (`F_E`):** not planned. Expression strings may be written in comments (spec 02 §7).
3. **Units and logic codes:** `A_W` is in seconds and `C_T` in milliseconds (spec 03 §4). `C_L` uses the `LogicalOperation` enum: AND = 1, OR = 2, NOT = 3, XOR = 4, NAND = 5, NOR = 6, NXOR = 7 (spec 03 §4.1).
4. **CI:** a GitHub Actions workflow builds and tests on Windows and Linux (spec 07 §3).

## WP2 decisions (JB, 2026-10-06)

1. **Ports are never picked because they are free.** Which device sits on which port is fixed by the wiring or the connection order. A port comes from the robot's port map (first-time setup, `.mrsp`), from an explicit address, or from a port scan that identifies the device (spec 04 §6).

## WP7 decisions (JB, 2026-10-08)

1. **Idle robots wait at their own layer** (spec 08 §3.1a). A later mode may land idle robots and bring them up to their layer only to hear from task dispatch, depending on the radio and on the signal at ground level.
2. **Point presets:** land, release a package (only when the robot carries it, it is the recipient's package, and the robot has landed), take a picture, orient the gimbal, flash the LEDs (spec 11 §3.1).
3. **Points of interest** are searched in a spiral with minimum overlap (spec 11 §3.5).

## WP8 decisions (JB, 2026-10-08)

1. **Split only what can be split.** A tree is decomposed down to units, the parts one robot takes; a sequence whose steps are bound to each other by affinity (a waypoint chain, land then release) is one unit (spec 12 §2.1).
2. **The binding is physical.** "Take a package, go to x, place the package" must not be decomposed: the package placed must be the one taken. Such tasks are written with each step bound to the previous one by `R_K`, so they stay whole (spec 12 §2.1).
3. **Exclusive mode** depends on the task definitions and the environment; the started-task claim rule is accepted (spec 12 §3.2).
4. **STA level with RTA-X** is expected for simple trees; it should later be compared on complex tasks and on safety, energy and distance too.
