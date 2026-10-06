# Spatial3D: 3D spatial task allocation for UAV groups

This project builds a group of flight-capable UAVs on the MRS framework. The UAVs allocate and execute 3D spatial tasks. The project also answers one question: **does a group of N UAVs finish a set of spatial tasks faster than one UAV flying a planned mission?**

Branch: `spatial-3d`. Status: **WP0**. The specification is written; the library skeleton goes in [`MRSlib/libmrs`](../../MRSlib/libmrs).

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
