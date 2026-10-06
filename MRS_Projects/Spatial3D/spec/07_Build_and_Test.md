# 07 Build and test

Status: **draft for WP0, updated in WP2**, spec version `0.1`. Plan reference: §3.2 (modules) and §11 (WP0).

## 1. Repository layout (branch `spatial-3d`)

The library lives in `MRSlib/` (JB, 2026-10-05); the project's documents, specification and experiments stay in `MRS_Projects/Spatial3D/`.

```
MRSlib/
  MRS/                     the legacy 2021 code, left untouched
  libmrs/                  the new library (WP0 creates the skeleton)
    CMakeLists.txt
    .clang-format
    core/                  MRS::Port, MRS::Device, MRS::Environment, MRS::Task, MRS::Comm, MRS::Protocol
      include/mrs/...      public headers; device/uav/ holds the UAV device classes (WP2)
      src/
    algorithms/            MRS::Algorithms
    platforms/webots/      WebotsPlatform (mrs_webots), built only when WEBOTS_HOME is set
    platforms/ardupilot/   later (WP10)
    behaviours/            .mrsb files
    tools/                 mrs_fmt (check a file, print its canonical form)
    tests/
      third_party/doctest.h
      protocol/            round-trip and error tests
      task/                conditions and executor (WP1)
      device/              device tree, ports, blocks, self model, with a mock platform (WP2)
MRS_Projects/Spatial3D/
  README.md
  docs/                    plan and design notes
  spec/                    this specification
    examples/              example protocol files (also test inputs)
  experiments/uav_spatial/ worlds, task sets, batch runner, analysis (WP6)
.github/workflows/spatial3d.yml
```

* The folder is `libmrs`, not `mrs`, because Windows file systems ignore case and `MRSlib/mrs` would be the same folder as the legacy `MRSlib/MRS`.
* Headers are still included as `#include <mrs/...>`.
* Classes are ported from `MRSlib/MRS` into `MRSlib/libmrs` piece by piece in WP1.

## 2. Build rules

| Item | Rule |
|---|---|
| Build system | CMake ≥ 3.20 |
| Language | C++17. No compiler extensions (`CMAKE_CXX_EXTENSIONS OFF`) |
| Compilers | MSVC 2022 (v143), GCC ≥ 11, Clang ≥ 14 |
| Warnings | MSVC `/W4 /permissive-`, GCC and Clang `-Wall -Wextra -Wpedantic`. Warnings are errors in CI |
| Dependencies | `core` and `algorithms`: the C++ standard library only. No Boost. `platforms/webots`: the Webots C++ API from `WEBOTS_HOME`. `platforms/ardupilot`: MAVLink C headers, vendored |
| Targets | `mrs_core`, `mrs_algorithms` (static libraries), `mrs_webots` (optional), `mrs_tests` |
| Options | `MRS_BUILD_TESTS` (ON), `MRS_BUILD_TOOLS` (ON), `MRS_WARNINGS_AS_ERRORS` (OFF; ON in CI), `MRS_BUILD_WEBOTS` (AUTO: ON when `WEBOTS_HOME` is set) |
| Exports | No DLL macros in `core`. The C API (later) is a separate target with its own export macro |
| Formatting | `.clang-format` at `MRSlib/libmrs/`, based on the existing code style (tabs, braces on the same line) |
| Text I/O | Number formatting and parsing use `<charconv>` (`std::to_chars`, `std::from_chars`), so results do not depend on the locale |

`std::from_chars` for floating point needs MSVC 2019 16.4+, GCC 11+ and a recent libc++; the compiler minimums above cover this.

## 3. Continuous integration

GitHub Actions workflow `.github/workflows/spatial3d.yml` (JB, 2026-10-05), triggered on pushes to `spatial-3d` and on pull requests, when they touch `MRSlib/libmrs/**`, `MRS_Projects/Spatial3D/spec/**` or the workflow itself:

* Jobs: `windows-latest` (MSVC) and `ubuntu-latest` (GCC).
* Steps: configure, build, `ctest --output-on-failure`.
* Webots is not installed in CI. The Webots platform is built and tested on a developer machine only.

## 4. Test framework

* **doctest** (single header, MIT licence), vendored at `MRSlib/libmrs/tests/third_party/doctest.h`.
* One test executable, `mrs_tests`, registered with CTest.
* Test data is read from `MRS_Projects/Spatial3D/spec/examples/` (the path is passed to the tests by CMake), so the examples in the spec and the tests cannot drift apart.

## 5. WP0 test suite

WP0 delivers the protocol layer (parser, writer, object model for every code in specs 01–06) and these tests.

### 5.1 Round trip

For every file in `spec/examples/*.mrs?`:

1. **Parse** the file. It MUST parse without errors.
2. **Write** it in canonical form (spec 01 §3.5).
3. **Compare** with `spec/examples/canonical/<name>`: byte-identical. `.mrsm` message logs use the message form (spec 01 §3.5).
4. **Parse** the canonical output and compare the object tree with step 1: structurally equal (same codes, slots, values, children).
5. **Write** again: byte-identical to step 2.

### 5.2 Error tests

`spec/examples/invalid/` holds one file per error class in spec 01 §5, with the expected error class and byte offset (spec 01 §5) in its first comment line, for example `# expect: reference_error 42`. The second line says what is wrong. The files are checked out with LF line endings on every platform (`.gitattributes`), so the offsets hold. Each MUST fail with that class at that offset.

| File | Error |
|---|---|
| `lex_bad_char.mrst` | lexical error |
| `lex_nan.mrst` | `nan` as a number |
| `code_unknown.mrst` | `T_Z` |
| `code_reserved.mrst` | `T_G` |
| `code_legacy_action.mrst` | `A_D` |
| `slot_count.mrst` | `C_P3` with 5 values |
| `slot_type.mrst` | a label where a number is expected |
| `ref_missing.mrst` | a referenced sub-record is absent |
| `ref_order.mrst` | sub-records in the wrong order |
| `ref_duplicate.mrst` | the same label twice in one body |
| `range_reversed.mrst` | `A_3..1` |
| `version_major.mrst` | `@: MRS 1.0/` |
| `trailing.mrst` | an unterminated record at the end |
| `map_nested.mrst` | `A_MAP` inside `A_MAP` |
| `fn_arity.mrst` | `A_FN A_PXY` with one function |
| `slot_logic_op.mrst` | `C_L 8`, not a `LogicalOperation` |
| `slot_view_match.mrst` | `C_V` with a view other than `V_DET` or `V_PEER` |
| `slot_atomic_fn.mrst` | `A_FN` inside a `T_A` |
| `slot_port_any.mrsp` | `P_A` with address `any` |

### 5.3 Action packing

Fixed vectors for spec 02 §2:

| Code and values | Expected `arg` (hex) |
|---|---|
| `A_PXY 1 -2` | `3F800000C0000000` |
| `A_PZY 15 1.5` | `417000003FC00000` |
| `A_W 2.5` | `4004000000000000` |
| `A_L 3 16711680` | `0000000300FF0000` |
| `A_MOT -409` | `C079900000000000` |
| `A_N` | `0000000000000000` |

Plus: unpacking each `arg` gives back the values; `A_PXY 1e39 0` fails (outside the binary32 range).

### 5.4 Device tests (WP2)

`tests/device/` runs against `MockPlatform`, which lists the Webots Mavic 2 Pro device names and records writes and messages:

* `examples/mavic_webots.mrsd` with `examples/mavic_webots.mrsp` builds with no faults, every port from the map; the self model lists the expected actions, views, message mode, capacity and fields, and meets the static requirements of `uav_point_task.mrst`.
* Without the GNSS line, `pose.enu` (and what depends on it) is gone and the point task no longer fits.
* Without a port map, addresses come from the parameters, and the port map written from that assignment has the example map's addresses.
* Port assignment: `any` without an identity stays unresolved even when ports are free; a scan resolves an identity that matches exactly one port; the map comes before the address; unlisted ports are not present; an exclusive port given twice and bad map entries are build errors.
* A missing or unopened device makes its node unavailable (a fault, not an error), and the fcu follows its motors.
* Dispatch rules of spec 04 §4.2, sensor views through ports, and definition errors.

### 5.5 Out of scope for WP0

Condition evaluation, the executor, devices and the worldview get their own tests in WP1–WP3. WP0 only checks that every object can be read and written exactly.
