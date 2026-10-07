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
    core/                  MRS::Port, MRS::Device, MRS::Environment, MRS::Task, MRS::Robot, MRS::Comm, MRS::Protocol
      include/mrs/...      public headers; device/uav/ holds the UAV device classes (WP2) and the
                           flight control unit (WP4); robot/ the robot controller, safety
                           supervisor, resources and journal (WP4); comm/ messages,
                           transports and the messenger (WP5)
      src/
    algorithms/            MRS::Algorithms: task pool, peer table, allocators, the MRS layer
                           and the task issuer (WP5)
    platforms/webots/      WebotsPlatform (mrs_webots), built only when WEBOTS_HOME is set;
                           controllers/mrs_worldview_check: the WP3 ground-truth check;
                           controllers/mrs_uav and mrs_issuer: the WP5 team controllers
    platforms/ardupilot/   later (WP10)
    behaviours/            .mrsb files
    tools/                 mrs_fmt (check a file, print its canonical form);
                           mrs_port2021 (convert a 2021 task set, WP5)
    tests/
      third_party/doctest.h
      protocol/            round-trip and error tests
      task/                conditions and executor (WP1)
      device/              device tree, ports, blocks, self model, with a mock platform (WP2)
      world/               worldview pipeline, source selection, processors (WP3)
      flight/              QuadSim (a rigid-body test platform) and the single-UAV flight tests (WP4)
      team/                several UAVs over a simulated radio: messages, claims, RTA, dumps (WP5)
MRS_Projects/Spatial3D/
  README.md
  docs/                    plan and design notes
  spec/                    this specification
    examples/              example protocol files (also test inputs)
  experiments/uav_spatial/ ported 2021 task sets and the 5-UAV mission (WP5); worlds,
                           batch runner, analysis (WP6)
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
| `code_param_value.mrsd` | A parameter value `D_Q` that is not a known code |

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

### 5.5 Worldview tests (WP3)

* A synthetic flight (a climbing circle) through the full UAV chain: `pose.enu` and `alt.agl` match the truth of the last GNSS sample within 1e-4 m, `heading` within 0.03 rad, `vel.enu` within 0.1 m/s.
* Source selection: the heading falls back from the compass to the attitude when the compass goes stale, and back; the source order is per robot; a rangefinder and a barometer are preferred when present; without GNSS, `pose.enu` and what depends on it go stale.
* Removing or adding a processor changes nothing else; a robot without GNSS loses the processors that need it.
* Chain rules: order by needs, a field provided twice, provided and offered, and a cycle.
* The self model's processor list equals the chain built for the same views; device views from the mock platform reach the worldview.
* Peers, detections, `landed`, `airborne`, `home`, time series and worldview requirements.

In Webots, `platforms/webots/controllers/mrs_worldview_check` compares the same fields with supervisor ground truth (its README has the steps).

### 5.6 Flight tests (WP4)

`tests/flight/` flies one UAV in `QuadSim` (spec 08 §9) through the robot controller, with the Mavic definition and port map of the examples and the behaviour library:

* [`uav_single_flight.mrst`](examples/uav_single_flight.mrst): liftoff, fly-to (by `flyto.layered`), an LED flash and a landing all succeed; the UAV lands within 1.5 m of the target without a crash; the journal is valid protocol text with `start`, `J_T`, `J_K` and `end`.
* Each fly-to variant (`layered`, `direct`, `altitude_first`, `velocity`), made the highest priority with `SetPriority`, reaches its target.
* The safety supervisor clamps positions, take-off altitude and velocities; a task target outside the shrunk fence fails as `IMPOSSIBLE` and the next task runs.
* Wind pushes the UAV out of a fence: GEOFENCE, return home, land at home, then the task resumes and the list finishes.
* Low battery with enough energy: the current task finishes, the next is blocked, the UAV lands at home and journals `swap_land`, above the reserve.
* Low battery without enough energy: return at once with the task preempted; a new controller on a full battery reads the journal (with a partly written last record), lifts off and finishes the unfinished tasks. A journal of another mission is refused.
* Critical battery: emergency landing where the UAV is, no return home.
* Energy: after a calibration flight, the estimate for an out-and-back leg with a hold is within 15 % of the battery drop; RLS recovers known model parameters.

### 5.7 Team tests (WP5)

`tests/team/` runs several QuadSim UAVs, each with its own MRS layer, over a simulated broadcast medium, with a task issuer as the operator (spec 09 §9):

* Messages encode and decode; the messenger drops its own, duplicate, foreign-mission and wrongly addressed messages.
* Claims: the better score wins, ties go to the lower robot number, expired claims are dropped.
* The 2021 port gives the expected ids, positions and order; the issuer sends each task once at its time.
* Open RTA: 5 UAVs finish ported task set 1 with no stalls, at least 4 of them do tasks, and the mission file equals the issuer's header.
* Exclusive RTA: every task is done exactly once.
* A UAV built without LEDs dumps the LED tasks as static and does none of them; the others do them. With two such UAVs and no LED-capable UAV left for a task, the task becomes `IMPOSSIBLE`.
* A task whose robot went silent is taken back by another robot.

### 5.8 Out of scope for WP0

Condition evaluation, the executor, devices and the worldview get their own tests in WP1–WP3. WP0 only checks that every object can be read and written exactly.
