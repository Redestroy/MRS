# 17 Track files and the MRS dot viewer

Status: spec version `0.1`, 2026-10-14. Requested by JB (2026-10-14): a tool outside Webots that shows the 3D movement of the drones, so a team spread over a large area can be followed in one view and replayed.

## 1. Scope

* **Track files** (§2): the poses of the robots over time, as CSV, with the mission and the task events in comment lines. The Webots supervisor (`mrs_issuer`) and the QuadSim runner (`mrs_experiment`) write them.
* **Task markers** (§3): the spatial conditions of the tasks, read from the protocol files the issuer already uses (`.mrsl` timelines, `.mrst` task files, `.mrs` mission headers). No new task format.
* **The MRS dot viewer** (§4): `mrs_dot_viewer`, an SFML window that replays one track file with its task files. It needs no other file: no font, image or configuration.

Code: `MRS::Replay` in `replay/` (`mrs_replay`, no SFML, always built and tested); the viewer in `tools/dot_viewer/` (optional target, §4.4).

## 2. Track files

### 2.1 Layout

A text file, UTF-8, lines ending in LF (CR LF is read too). Blank lines are ignored. A line that starts with `#` is a comment; two comment forms carry data (§2.3). The first other line is the **header row**; every later line is a **row**.

```
# MRS track 0.1 (spec 17)
# mission H: H_M m1 56.9496 24.1052 10 -100 100 -100 100 60 20 5 120 3 1 -4 -5 0 2 0 -5 0 3 4 -5 0/
t,r1.x,r1.y,r1.z,r1.roll,r1.pitch,r1.yaw,r2.x,r2.y,r2.z,r2.roll,r2.pitch,r2.yaw
0.000,-4.000,-5.000,0.100,0.0000,0.0000,1.5708,0.000,-5.000,0.100,0.0000,0.0000,1.5708
0.104,-4.000,-5.000,0.100,0.0000,0.0000,1.5708,0.000,-5.000,0.100,0.0000,0.0000,1.5708
# event 1.000 dispatch op.1
```

### 2.2 Header row and rows

* Cells are separated by `,`; spaces around a cell are ignored.
* The first column is `t`: seconds of simulation time, the same clock as the timeline's `L_D` times. `t` MUST increase strictly from row to row.
* Every other column is `<robot>.<axis>`, with `axis` one of `x`, `y`, `z` (ENU metres, spec 00 §3) or `roll`, `pitch`, `yaw` (radians; the body-to-world rotation is `Rz(yaw) Ry(pitch) Rx(roll)`). A robot needs `x`, `y` and `z`; its angles are optional, and a missing angle is 0. The robot name is everything before the last `.`. Column order is free.
* Columns without a `.` are taken in groups of three as `x`, `y`, `z` of robots `1`, `2`, … (for example `t,x1,y1,z1,x2,y2,z2`), so a plain position log also loads.
* Each row has exactly one cell per column. A robot's cells are all numbers, or all empty: empty means the robot is not in the scene at that time (not yet spawned, removed).
* Writers in libmrs write the time with 3 decimals, positions with 3 and angles with 4, and drop a row whose time is not later than the last row's at that precision.

### 2.3 Data comments

| Comment | Meaning |
|---|---|
| `# mission <record>` | The mission header (`H_M`, spec 06 §6) in compact form. It gives the geo reference for `C_G` markers and the fence the viewer draws |
| `# event <t> dispatch <task>` | The issuer sent the task (or the leaf or unit of a tree) at `t` |
| `# event <t> done <task> <robot>` | The first `M_DONE` for the task, from that robot |
| `# event <t> failed <task> <reason>` | The task failed (`M_FAIL`, or `TREE` for a failed tree root) |

Task ids are those of the issuer: a root's own id, `p.i` for the nodes of a tree (spec 11 §2.1). Events need not be in time order relative to the rows. Any other comment is free text. An event with another kind, or a mission record that does not parse, is an error.

### 2.4 Errors

The reader stops at the first problem and reports its line: no header row, a header without `t` first, an unknown axis, a robot without `x`, `y` or `z`, a column twice, a row with the wrong number of cells, a cell that is not a number, a robot with some cells empty, `t` not increasing, or a bad data comment.

### 2.5 Writers

**Webots.** `mrs_issuer` (spec 09 §6) logs every `Robot` node at the top of the scene tree except itself, by its `name` field, with the pose from the supervisor API (`getPosition`, `getOrientation`). Arguments, after the positional ones:

* `--track <file>`: where to write; default `<results without .csv>_track.csv` next to the results (`worlds/mavic_team_track.csv` in the team world); `none` turns it off;
* `--track-period <s>`: seconds between rows, default 0.1.

It writes the mission header first, a row every period, the events as the issuer sees them, and a last row when every task has ended. The file is flushed after each row, so a stopped simulation keeps its track.

**QuadSim.** `mrs_experiment run ... --track <file> [--track-period <s>]` writes the track of exactly one run (one set, one seed, one condition), with the true poses of the simulator. `--set <file.mrsl>` names one timeline file. Example:

```
mrs_experiment run --set taskset1.mrsl --cond G-RTA:5 --seeds 1-1 --out run.csv --track run_track.csv
```

## 3. Task markers

A **marker** is one spatial condition of a task: `C_P3` (`x y z tol_xy tol_z`) as written, or `C_G` (`lat lon alt tol_xy tol_z`) converted to ENU through the mission's geo reference. Other conditions have no place and get no marker.

* Files: `.mrsl` timelines (each `L_D t` gives its tasks the dispatch time `t`), `.mrst` task files and top-level `T` records (dispatch time 0), and any file with a `H_M`. Several files may be given; their markers are added up.
* Every `C_P3` or `C_G` under a task belongs to it: start, end and until conditions, runtime requirements and behaviours alike. Identical markers of one task are kept once.
* Ids follow the decomposer (spec 11 §2.1): child `i` of a `T_S`, `T_L` or `T_O` with id `p` is `p.i`, so a marker carries the id the issuer reports for its leaf. A root with id `0` is named `task<n>`.
* Geo reference: the track's `# mission`, else the first `H_M` among the task files. Without one, `C_G` markers are skipped with a warning.
* Events (§2.3) are applied to markers: a `dispatch` event for the marker's id replaces the timeline's time (the issuer's actual dispatch); `done` and `failed` events for the marker's id or a node above it end the marker. The first of each kind counts.

A marker is **hidden** before its dispatch time, **open** until it is done or failed, then **done** or **failed**.

## 4. The MRS dot viewer

```
mrs_dot_viewer <track.csv> [task files .mrsl/.mrst/.mrs ...] [--trail S] [--shape sphere|quad]
               [--size M] [--speed X] [--at S] [--follow ROBOT] [--screenshot FILE] [--window WxH]
```

| Option | Default | Meaning |
|---|---|---|
| `--trail S` | 20 | Seconds of fading track behind each robot; 0 turns it off |
| `--shape` | `sphere` | `sphere`: a ball per robot. `quad`: an X frame with rotors and a nose line, which shows the attitude |
| `--size M` | 0.6 | Robot radius in metres. A robot is never drawn smaller than 7 px across, so far robots stay visible |
| `--speed X` | 1 | Replay speed |
| `--at S` | track start | Start time |
| `--follow ROBOT` | none | Start with the camera following that robot |
| `--screenshot FILE` | | Render one frame at `--at` to an image (`.png`, `.jpg`, `.bmp`) and exit |
| `--window WxH` | 1280x800 | Window size |

Exit codes: 0, 1 for an unreadable track, 2 for bad arguments. Task files that cannot be read or parsed are reported on standard error and skipped.

### 4.1 Scene

* ENU, z up. A ground grid on `z = 0` (5, 10, 20 or 50 m by extent) with the ENU axes at the origin (x red, y green, z blue), and the fence box from the mission header.
* **Robots**: a ball (or quad frame) in the robot's colour, its name beside it, a faint drop line to its point on the ground, and a **fading track**: the robot's path over the last `trail` seconds, opaque at the robot and transparent at the end. Positions and angles are interpolated linearly between rows (angles the short way round).
* **Tasks**: each marker is a **flagged ball**: a ball at the condition's point with a flag on a short pole and the task id. The **condition volume** is drawn around it: an ellipsoid with radius `tol_xy` across and `tol_z` up (a sphere when they are equal), as three rings and a translucent fill. Colours: orange while open, green when done (label `id (robot)`), red when failed; ended markers are dimmed.

### 4.2 Controls

The replay bar at the bottom has a **play/pause button** and a **slider** to seek. Ticks above the slider are dispatches, ticks below are completions (green) and failures (red). Beside it: time, speed and trail length.

| Input | Action |
|---|---|
| Space, play button | Play / pause (at the end, play restarts) |
| Left / Right | 1 s back / forward; with Shift 10 s |
| Up / Down | Speed ×2 / ÷2 (⅛ to 64) |
| Home / End | Start / end |
| `[` / `]` | Shorter / longer trail |
| 1–9, 0 | Follow robot 1–9 (legend order) / free camera |
| F | Fit the scene |
| T | Look from the top |
| L | Labels on / off |
| G | Ground lines (grid, drop lines) on / off |
| Q | Sphere / quad shape |
| H | Help |
| Left drag / right drag / wheel | Orbit / pan / zoom |

The legend (top left) lists the robots by number with their altitude, and counts done, open and failed tasks.

### 4.3 Rendering

SFML 3 (graphics module) draws the projected scene in 2D: an orbit camera with perspective projection (50° vertical field of view), lines clipped at the near plane, balls as shaded sprites sorted far to near. Text uses a built-in bitmap of DejaVu Sans Mono (`tools/dot_viewer/src/FontData.h`, made by `tools/dot_viewer/tools/make_font.py`), so no font file is needed.

### 4.4 Build

The viewer is off by default, so the library and CI need no SFML:

```
cmake -S MRSlib/libmrs -B build -DMRS_BUILD_VIEWER=ON
cmake --build build --target mrs_dot_viewer --config Release
```

CMake fetches SFML (`MRS_SFML_VERSION`, default 3.0.2) from GitHub and links it statically, so `mrs_dot_viewer` runs without SFML DLLs. On Windows it needs nothing else; on Linux it needs the X11, Xrandr, Xcursor, Xi, udev, OpenGL and FreeType development packages (Ubuntu: `libxrandr-dev libxcursor-dev libxi-dev libudev-dev libgl1-mesa-dev libfreetype-dev`). The CI job `viewer` builds it on Windows and Linux and renders a QuadSim track on Linux.

## 5. Tests

`tests/replay/test_replay.cpp`: a written track reads back (mission, poses, events) and drops a row that is not later; interpolation with yaw across ±π; bare `x1,y1,z1` columns; empty cells; every error of §2.4 with its line; the 15 markers of the Webots task set with their dispatch times; leaf ids of the tree example; `C_G` with and without a geo reference; events applied to a tree's markers; a QuadSim run that writes its track, in which every task is done and the robot that did it was at the marker when it did.

## 6. Not in version 0.1

* Markers for areas and lines as such (a coverage tree shows its waypoints, one marker per leg).
* Robots logged from the robots' own estimates (the Webots track is the simulator's truth).
* Several tracks at once, and live viewing while a simulation runs (the file is flushed per row, so reloading shows progress).
