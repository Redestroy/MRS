# Spatial3D: 3D spatial task allocation for UAV groups

This project builds a group of flight-capable UAVs on the MRS framework. The UAVs allocate and execute 3D spatial tasks. The project also answers one question: **does a group of N UAVs finish a set of spatial tasks faster than one UAV flying a planned mission?**

Branch: `spatial-3d`. Status: **WP0, specification**. No library code exists yet.

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

* In the repository, everything for this project is under `MRS_Projects/Spatial3D/`.
* On JB's PC, the same tree is mirrored at `E:\Claude\MRS_Projects\Spatial3D\`.

## Open questions for WP0

Each one has a default that the documents already use, so work can continue.

1. **Library location.** The new library goes in `MRS_Projects/Spatial3D/mrs/` (default) or in `MRSlib/` next to the old code?
2. **Expression functions (`F_E`).** Is a parsable expression string such as `"0.8*(target.x - pose.enu.x)"` needed now? Default: no. Version 0.1 has linear (`F_L`) and registry (`F_X`) functions plus sum, product and clamp, and `F_E` is reserved.
3. **Legacy units for the converter.** In the old strings, what unit did `C_T 3000023` and `A_W 30` use (milliseconds, simulation steps)? And what did the number in `C_L 1 C_1 C_2` mean (1 = OR)? Default: milliseconds, and 0 = AND, 1 = OR.
4. **CI.** Add a GitHub Actions workflow that builds and tests on Windows and Linux for every push to `spatial-3d`? Default: yes.
