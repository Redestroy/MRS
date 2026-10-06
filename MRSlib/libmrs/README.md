# libmrs

The new MRS library (C++17, CMake, no Boost). It is specified by [`MRS_Projects/Spatial3D/spec`](../../MRS_Projects/Spatial3D/spec). The legacy 2021 code stays in [`../MRS`](../MRS) and is ported piece by piece from WP1 on.

Status: **WP1** (core repair). It contains the protocol layer, the task core and a minimal worldview:

| Path | What it is |
|---|---|
| `core/include/mrs/protocol/Record.h` | Generic object model: a record is a code, typed slots and its sub-records |
| `core/include/mrs/protocol/Parser.h` | Parser for spec 01, with the slot list of every code in specs 02–06 and the error classes of spec 01 §5 |
| `core/include/mrs/protocol/Writer.h` | Writer for the canonical form (spec 01 §3.5) |
| `core/include/mrs/device/Action.h` | Action registry and 64-bit argument packing (spec 02 §2, §4) |
| `core/include/mrs/world/` | `Worldview` (fields, freshness; WP3 extends it) and `GeoReference` (WGS-84 to ENU) |
| `core/include/mrs/task/Condition.h` | The condition family of spec 03 §4, three-valued, with target binding |
| `core/include/mrs/task/Function.h` | Functions of spec 02 §7 and the `FunctionRegistry` with the UAV functions |
| `core/include/mrs/task/TaskAction.h` | Leaf, combined (`A_MAP`) and parametric (`A_FN`) actions |
| `core/include/mrs/task/Task.h` | `Task`, `ATask`, `ParametricATask`, `Behaviour`, `ComplexTask`, requirements |
| `core/include/mrs/task/TaskFactory.h` | Builds tasks from protocol records |
| `core/include/mrs/task/BehaviourLibrary.h` | `Find` and `Populate` for `.mrsb` files |
| `core/include/mrs/task/TaskExecutor.h` | The executor contract of spec 03 §8 |
| `tools/mrs_fmt.cpp` | `mrs_fmt <file>`: checks a file and prints its canonical form |
| `tests/task/` | WP1: task strings run tick by tick against a scripted worldview and a toy UAV |
| `tests/protocol/` | The WP0 suite of spec 07 §5: round trip against `spec/examples/canonical/`, one file per error class in `spec/examples/invalid/`, action packing vectors, and parser rule tests |

Build and test:

```
cmake -S MRSlib/libmrs -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

On Windows with Visual Studio, add `-C Debug` (or `Release`) to the build and ctest commands.
