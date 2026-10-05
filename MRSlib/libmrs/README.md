# libmrs

The new MRS library (C++17, CMake, no Boost). It is specified by [`MRS_Projects/Spatial3D/spec`](../../MRS_Projects/Spatial3D/spec). The legacy 2021 code stays in [`../MRS`](../MRS) and is ported piece by piece from WP1 on.

Status: **WP0 skeleton**, for review. It contains the protocol layer only:

| Path | What it is |
|---|---|
| `core/include/mrs/protocol/Record.h` | Generic object model: a record is a code, typed slots and its sub-records |
| `core/include/mrs/protocol/Parser.h` | Parser for spec 01, with the slot list of every code in specs 02–06 and the error classes of spec 01 §5 |
| `core/include/mrs/protocol/Writer.h` | Writer for the canonical form (spec 01 §3.5) |
| `core/include/mrs/device/Action.h` | Action registry and 64-bit argument packing (spec 02 §2, §4) |
| `tools/mrs_fmt.cpp` | `mrs_fmt <file>`: checks a file and prints its canonical form |
| `tests/` | doctest smoke tests. The full suite of spec 07 §5 follows after the review |

Build and test:

```
cmake -S MRSlib/libmrs -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

On Windows with Visual Studio, add `-C Debug` (or `Release`) to the build and ctest commands.
