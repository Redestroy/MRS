# 07 Build and test

Status: **draft for WP0**, spec version `0.1`. Plan reference: §3.2 (modules) and §11 (WP0).

## 1. Repository layout (branch `spatial-3d`)

```
MRS_Projects/Spatial3D/
  README.md
  docs/                    plan and design notes
  spec/                    this specification
    examples/              example protocol files (also test inputs)
  mrs/                     the new library (WP0 creates the skeleton)
    CMakeLists.txt
    core/                  MRS::Port, MRS::Device, MRS::Environment, MRS::Task, MRS::Comm, MRS::Protocol
      include/mrs/...      public headers
      src/
    algorithms/            MRS::Algorithms
    platforms/webots/      built only when Webots is found
    platforms/ardupilot/   later (WP10)
    devices/uav/
    behaviours/            .mrsb files
    tests/
      third_party/doctest.h
      protocol/            round-trip and error tests
  experiments/uav_spatial/ worlds, task sets, batch runner, analysis (WP6)
```

The legacy `MRSlib/` tree is left untouched. Classes are ported from it into `mrs/` piece by piece in WP1.

## 2. Build rules

| Item | Rule |
|---|---|
| Build system | CMake ≥ 3.20 |
| Language | C++17. No compiler extensions (`CMAKE_CXX_EXTENSIONS OFF`) |
| Compilers | MSVC 2022 (v143), GCC ≥ 11, Clang ≥ 14 |
| Warnings | MSVC `/W4 /permissive-`, GCC and Clang `-Wall -Wextra -Wpedantic`. Warnings are errors in CI |
| Dependencies | `core` and `algorithms`: the C++ standard library only. No Boost. `platforms/webots`: the Webots C++ API from `WEBOTS_HOME`. `platforms/ardupilot`: MAVLink C headers, vendored |
| Targets | `mrs_core`, `mrs_algorithms` (static libraries), `mrs_webots` (optional), `mrs_tests` |
| Options | `MRS_BUILD_TESTS` (ON), `MRS_BUILD_WEBOTS` (AUTO: ON when `WEBOTS_HOME` is set) |
| Exports | No DLL macros in `core`. The C API (later) is a separate target with its own export macro |
| Formatting | `.clang-format` at `mrs/`, based on the existing code style (tabs, braces on the same line) |
| Text I/O | Number formatting and parsing use `<charconv>` (`std::to_chars`, `std::from_chars`), so results do not depend on the locale |

`std::from_chars` for floating point needs MSVC 2019 16.4+, GCC 11+ and a recent libc++; the compiler minimums above cover this.

## 3. Continuous integration

GitHub Actions workflow `.github/workflows/spatial3d.yml`, triggered on pushes and pull requests that touch `MRS_Projects/Spatial3D/**`:

* Jobs: `windows-latest` (MSVC) and `ubuntu-latest` (GCC).
* Steps: configure, build, `ctest --output-on-failure`.
* Webots is not installed in CI. The Webots platform is built and tested on a developer machine only.

## 4. Test framework

* **doctest** (single header, MIT licence), vendored at `mrs/tests/third_party/doctest.h`.
* One test executable, `mrs_tests`, registered with CTest.
* Test data is read from `MRS_Projects/Spatial3D/spec/examples/`, so the examples in the spec and the tests cannot drift apart.

## 5. WP0 test suite

WP0 delivers the protocol layer (parser, writer, object model for every code in specs 01–06) and these tests.

### 5.1 Round trip

For every file in `spec/examples/*.mrs?`:

1. **Parse** the file. It MUST parse without errors.
2. **Write** it in canonical form (spec 01 §3.5).
3. **Compare** with `spec/examples/canonical/<name>`: byte-identical.
4. **Parse** the canonical output and compare the object tree with step 1: structurally equal (same codes, slots, values, children).
5. **Write** again: byte-identical to step 2.

### 5.2 Error tests

`spec/examples/invalid/` holds one file per error class in spec 01 §5, with the expected error class and byte offset in its first comment line, for example `# expect: reference_error 42`. Each MUST fail with that class at that offset.

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

### 5.4 Out of scope for WP0

Condition evaluation, the executor, devices and the worldview get their own tests in WP1–WP3. WP0 only checks that every object can be read and written exactly.
