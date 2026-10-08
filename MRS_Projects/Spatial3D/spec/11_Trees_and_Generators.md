# 11 Tree tasks across robots and task generators

Status: **WP7**, spec version `0.1`. Plan reference: §8.7 (tree tasks), §9.2 (generators), §11 (WP7). Builds on specs [03](03_Tasks_and_Execution.md) §6 (complex tasks, decomposition), [08](08_Robot_and_Flight.md) (robot controller) and [09](09_MRS_Layer.md) (MRS layer, task issuer).

## 1. Scope

WP7 lets a team share one tree task, and lets an operator describe work as geometry:

* the **decomposer** turns a tree into leaves with ids, precedence, gates and affinity (§2.1, §2.2);
* the **MRS layer** keeps the leaves in its task pool, blocks and unblocks them, derives the state of every complex node, and checks end conditions (§2.3 to §2.5);
* the **task issuer** follows the leaves and reports the root (§2.5);
* **generators** turn points, lines, areas and points of interest into tasks (§3);
* **GeoJSON** input and the `mrs_generate` tool write a timeline from an operator's map (§4);
* the devices the point presets need: camera gimbal, camera and payload bay (spec 04 §3.1, spec 02 §4.2a).

JB's decisions for WP7 (2026-10-08):

1. Idle robots wait at their own layer (spec 08 §3.1a). A later mode may land them and bring them up only to hear from task dispatch.
2. A point generator with preset tasks: land; release a package (only if the robot carries it, it is the recipient's package, and the robot has landed); take a picture; orient the gimbal; flash the LEDs (§3.1).
3. A points-of-interest generator that searches the area around each point in a spiral with minimum overlap (§3.5).

Code: `MRS::Algorithms` (`TaskTree.h`, the MRS layer and issuer) in `algorithms/`; `MRS::Generators` in the new `generators/` library (`mrs_generators`, which depends on `mrs_core`); `tools/mrs_generate.cpp`.

## 2. Tree tasks across robots

### 2.1 Decomposer

`Decompose(root)` returns a `TaskTree`: every node by id, the leaves in depth-first order, and per leaf:

| Member | Meaning |
|---|---|
| `task` | The leaf's own record, with its id (`p.i`, spec 03 §6.2 rule 1) and its effective priority (the product of the priorities on its path, rule 7) written in |
| `after` | Nodes that must be `DONE` first (rule 3: the previous sibling under each `T_S` ancestor whose later child holds this leaf first) |
| `gates` | Start conditions of complex ancestors that must be TRUE first (rule 6), for the first leaves of those ancestors only |
| `affinity` | The id in the leaf's `R_K`, or empty (rule 8) |

`FirstLeaves(tree, id)` are the leaves that can start a node: the node itself for a leaf, the first leaves of child 1 for a `T_S`, and the first leaves of every child for a `T_L` or `T_O`. `LeavesUnder(tree, id)` are all its leaves.

The root MUST carry an id (not `0`). Reserved codes (`T_D`, `T_G`, `T_U`), a complex task without children and a `T_L` whose `k` is more than its children throw `DecomposeError`. The result depends only on the record, so every robot and the issuer get the same tree.

### 2.2 Gates, not merged start conditions

Precedence and parent start conditions are kept beside the leaf, as `after` and `gates`. They are not merged into the leaf's start condition (spec 03 §6.2 amendment): a merged `C_L` or `C_S` start has no behaviour, so the leaf could never start its own fly-to. The leaf's own start condition stays as written and is fulfilled by the behaviour library as usual (spec 03 §7).

### 2.3 Leaves in the task pool

When a robot receives a tree (`M_TASK` with a `T_S`, `T_L` or `T_O` record), the MRS layer decomposes it, builds every leaf, and adds each leaf to its pool as an ordinary entry. A tree that fails to decompose, or that has a leaf the robot cannot load, is ignored by that robot. The static dump rule (spec 06 §5) applies to each leaf on its own.

Each tick, before allocation, `UpdateTrees`:

1. derives the state of every complex node (§2.4). A node that has just ended is checked once: if it is `DONE`, has an end condition other than `C_N`, and this robot finished its last leaf, the robot evaluates the end condition; FALSE (or a condition it cannot build) sets the node `FAILED` and sends `M_FAIL <node id> END_NOT_MET`. Then every leaf under the ended node that is not finished is `CANCELLED`;
2. sets each `AVAILABLE` or `BLOCKED` leaf to `BLOCKED` when it is **gated**, else `AVAILABLE`. A leaf is gated while:
   * a node in `after` is not `DONE`;
   * a condition in `gates` is not TRUE on this robot's worldview now;
   * it has an affinity partner in the tree that is not `DONE` by this robot (`done_by` is this robot's name).

`BLOCKED` entries are not eligible for any allocator (spec 09 §4). Affinity is per robot: the partner's robot sees the leaf `AVAILABLE`, the others see it `BLOCKED`.

Every received leaf starts `BLOCKED` and is unblocked by the first `UpdateTrees`. `M_DONE`, `M_FAIL` and `M_DUMP` for leaves are handled as for any task. An `M_FAIL` for a complex node id (an end condition another robot found FALSE) sets that node's state.

### 2.4 Derived states

| Node | `DONE` when | `FAILED` when |
|---|---|---|
| `T_S` | all children are `DONE` | any child has failed |
| `T_L` | `k` children are `DONE` (`k = 0`: all) | fewer than `k` children can still be `DONE` |
| `T_O` | one child is `DONE` | all children have failed |

A child "has failed" when it is `FAILED`, `IMPOSSIBLE` or `CANCELLED`. Otherwise a node is `AVAILABLE`. An override (an end condition found FALSE) wins over the derived state.

### 2.4a Runtime conditions before selection (WP7 choice, not asked)

An allocator does not select an entry whose runtime condition (`R_C`, spec 03 §5) is FALSE on the robot's worldview now; UNKNOWN does not rule it out. Before WP7 a runtime condition was checked only from `STARTED` (spec 03 §8.5), so a robot carrying another package flew to a release point and failed there. The check is `AllocatorContext::runnable` and applies to every allocator.

JB (2026-10-08) confirmed this for now: a robot skips a task whose runtime condition is FALSE as long as no behaviour could make it TRUE. *Later:* when a behaviour can fulfil the condition (for example, fetch the package first), the task stays eligible, and its size estimate includes the cost of that behaviour.

### 2.5 The issuer

When the issuer dispatches a tree root it decomposes it too, and follows each leaf as an `IssuedTask` (`Leaves()`). The root's entry in `Tasks()` is `done` (with the stamp) when the tree is `DONE`, and `failed` with reason `TREE` when it fails. A root is never part of an `M_PLAN` (spec 10 §2.3); G-C does not plan tree leaves in version 0.1.

### 2.6 One robot alone

`MrsConfig::split_trees` (default on) can be turned off, and the robot then runs each tree as one complex task (spec 03 §6.1). The single-UAV conditions of spec 10 (S1, S1\*) run with it off, so the oracle form (spec 10 §3.3) keeps its meaning: one robot flies to the task, waits for the release and does it.

## 3. Generators

Generators write protocol records (spec 03). Inner ids are written as `0` and affinity ids as the decomposer will assign them. All positions are ENU metres (spec 00 §3); `z` is the flight altitude above the reference.

### 3.1 Points (`PointTask`)

One point and one preset. Each preset is a `T_A` with the start condition `C_P3 x y z tol_xy tol_z 0 -1` (default tolerances 1 m and 0.5 m), except `release`.

| Preset | Actions | Notes |
|---|---|---|
| `leds` | `A_L mask colour`, `A_W d`, `A_L mask 0`, `A_W d` | Default mask 3 (both Mavic LEDs), colour `0xFF0000`, `d` = `duration` (3 s) |
| `hover` | `A_W d` | |
| `land` | `A_LD descent 0` | Lands at the point; `z` is the approach altitude |
| `picture` | `A_GMB pitch yaw`, `A_W min(d, 1)`, `A_CAM 1 0` | Pitch −π/2 (straight down) by default; the wait lets the gimbal settle |
| `gimbal` | `A_GMB pitch yaw` | |
| `release` | a `T_S` of two leaves, below | Package id from `package` |

The release tree (JB, 2026-10-08):

1. `p.1`: fly to the point and land (`A_LD`), with the runtime requirement `R_C C_m payload.id eq <package> 0.5`. Only a robot that carries the package may take it (§2.4a), and it fails if the package is gone before it lands.
2. `p.2`: start `C_? landed T`, requirement `R_K p.1`, actions `A_REL <package>`, `A_W 1`. The robot that landed releases, and only on the ground. `A_REL` itself fails if the bay holds another package.

The point's `z` MUST be a flight altitude: a target below `min_alt` is `IMPOSSIBLE` (spec 08 §3.2). GeoJSON points without an altitude get 15 m (§4.2).

### 3.2 Waypoint chains (`ChainTask`, `PathTask`)

A chain is a `T_S` over the waypoints, flown by one robot:

* leaf 1 is a `T_A` with start `C_P3` at the first waypoint and action `A_N`, so the robot takes off, climbs to its layer and goes there with its usual fly-to;
* leaf `i > 1` is a **level leg**, a `T_B` that holds a position setpoint until the waypoint is reached, with affinity to leaf `i − 1`:

```
T_B 0 1 0 C_1 C_2 C_3 R_1 T_1      start: C_? airborne T; end: C_N; until: C_P3 at waypoint i
R_1: R_K <chain id>.<i − 1>
T_1: T_P 0 1 0 C_N C_N A_MAP{any A_PXY x y, any A_PZY z yaw} A_N
```

`yaw` faces along the leg. A leg does not climb to the layer between waypoints: it flies straight at the waypoints' altitude. Tolerances: 1.5 m horizontal, 1 m vertical.

`PathTask` is a line string as one chain.

### 3.3 Area coverage (`CoverageTask`)

`CoverageTracks` lays boustrophedon tracks across the polygon:

* tracks run along the polygon's longest edge, or at `heading` (rad ENU) when given;
* neighbouring tracks are `footprint × (1 − overlap)` apart (defaults 20 m and 0.2), centred across the polygon;
* each track spans the first and last crossing of the boundary, so the polygon is treated as convex; every other track runs the other way.

`CoverageTask` cuts the tracks into `cells` (3) groups of neighbouring tracks of about equal length (track length + one footprint for the turn) and returns a `T_L` (`k = 0`) of one chain per cell. Within a cell each track starts at the end nearer the previous one. Each cell is flown by one robot (affinity); the cells are flown in parallel.

### 3.4 Perimeter (`PerimeterTask`)

The closed boundary at `altitude`, with no leg longer than `spacing` (25 m), back to the first corner. With `arcs = 1` it is one chain; with more, the ring is cut into `arcs` arcs of about equal length that share their end points, in a `T_L` (`k = 0`).

### 3.5 Points of interest (`SearchTask`)

`SpiralWaypoints(centre)` is an Archimedean spiral `r = a·θ` outwards from the point, with `2π·a = footprint × (1 − overlap)` (defaults 10 m and 0.1): neighbouring turns overlap by exactly `overlap`, the minimum that leaves no gap. It starts with the centre, then half a turn out (`θ = π`, so the centre is seen once), and continues until `r` passes `radius` (30 m). Points are about `step` (6 m) apart along the spiral and no leg is longer than `step`, except the first, which is half a pitch. Altitude: the point's own, else `altitude` (15 m).

`SearchTask(points)` is the spiral chain of one point, or a `T_L` (`k = 0`) with one spiral chain per point.

### 3.6 Landing after a tree (`ThenLand`)

`ThenLand(id, tree, robots)` returns a `T_S` of the tree and a `T_L` (`k = 0`) with one landing leaf per robot:

```
T_A 0 1 0 C_? home T, C_N, R_I <robot>, A_LD 1 0, A_N
```

The tree keeps its shape as child `id.1`; its affinity ids move with it. Each robot lands at home, and only after every leaf of the tree is `DONE`.

### 3.7 Files

`TaskFile(tasks)` writes a `.mrst`; `Timeline(entries)` writes a `.mrsl` with each task at its dispatch time (spec 06 §7).

## 4. GeoJSON input

### 4.1 Reader

`ReadGeoJson(text, geo)` reads a `FeatureCollection`, a `Feature` or a bare geometry (RFC 7946) and returns one `Feature` per part: `Point`, `LineString` or `Polygon` with ENU points and the feature's `properties`. `MultiPoint`, `MultiLineString`, `MultiPolygon` and `GeometryCollection` give one feature per part. A polygon uses its outer ring without the closing corner; holes are ignored. Positions are `[lon, lat]` or `[lon, lat, alt]`; `alt` is metres above the reference (the mission header's `alt0`) and becomes `z` directly. Errors (bad JSON with its byte offset, unknown geometry, a position that is not 2 or 3 numbers, a ring of fewer than 3 corners) throw `JsonError`.

### 4.2 Features to tasks

`TasksFromGeoJson(features, options)` gives ids `<issuer>.<first_id>`, `.+1`, … in feature order. The `task` property picks the generator:

| Geometry | `task` | Generator | Properties (defaults in §3) |
|---|---|---|---|
| Point | `leds` (default), `hover`, `land`, `picture`, `gimbal`, `release` | §3.1 | `priority`, `duration`, `colour`, `pitch`, `yaw`, `package`; `altitude` overrides the position's |
| Point, MultiPoint | `search` | §3.5 | `group` (features with the same group are searched in one tree; default: each feature on its own), `altitude`, `radius`, `footprint`, `overlap`, `step` |
| LineString | `path` (default) | §3.2 | `altitude` (default 20 m when the line has none), `priority`, `tol_xy`, `tol_z` |
| Polygon | `coverage` (default) | §3.3 | `altitude`, `footprint`, `overlap`, `cells`, `heading`, chain properties |
| Polygon | `perimeter` | §3.4 | `altitude`, `spacing`, `arcs`, chain properties |

A Point without altitude gets 15 m. With `land_robots` set, every tree (`T_S` or `T_L`) is wrapped by `ThenLand` for those robots. An unknown `task` throws `JsonError`.

### 4.3 `mrs_generate`

```
mrs_generate <features.geojson> <mission header .mrs> [--at SECONDS] [--land r1,r2,...] [--issuer op] [--first N]
```

Reads the geo reference from the mission header (`H_M`, spec 06 §6), converts the features and writes a timeline to standard output with every task at `--at` seconds (default 1). Exit code 0 on success, 1 on an input error, 2 on bad arguments.

Examples in `experiments/uav_trees/`: `field.geojson` (the acceptance polygon), `points.geojson` (one feature per preset) and `search.geojson` (three points of interest in one group).

## 5. Tests (WP7)

`tests/tree/test_tree.cpp` (spec 07 §5.9):

* decomposition of the spec example (ids, precedence, first leaves), gates and affinity, the rejected trees;
* derived states of `T_S`, `T_L` and `T_O`, and the end-condition override;
* coverage spacing, direction and cells; spiral pitch, radius and leg length; perimeter arcs; every point preset and the release tree; `ThenLand`; GeoJSON parsing and its errors;
* `R_I`: only the named robot takes the task;
* idle at layer: a robot that finished a task at 32 m waits at its 20 m layer;
* the picture and gimbal presets drive the simulated camera and gimbal (gimbal pitch clamped);
* release: of three robots, only the one carrying package 7 takes it, lands and releases at the point; the others stay on the ground;
* **acceptance** (plan §11 WP7): `field.geojson` becomes a coverage tree with landings; three UAVs with MRS-RTA complete it, each cell is flown by one robot, all three cells are used, no leaf is done twice, every landing comes after the last strip, and every robot ends on the ground inside the fence.

## 6. Not in WP7

* G-C planning of tree leaves, and STA/LDTA/CBBA on trees (WP8 onwards).
* Concave polygons and holes in coverage (tracks span the outer crossings).
* Spiral search that stops on a detection (an end condition differing from the until condition, JB 2026-10-06 behaviour decision 2).
* The land-when-idle mode (spec 08 §3.1a).
