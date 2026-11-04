# 18 Package delivery

Status: **draft**, started 2026-11-04 from JB's delivery demo request. Plan: `/mnt/project-files/delivery/Package_Delivery_Plan.md` (project files). JB's answers of 2026-11-04 are applied: fixed-length sling with a passive hook and loop; the destination is read from the package at pickup; intermediate drops go to allowed drop areas; relaying carries all messages; recharge at pads, swap as fallback.

## 1. Scope

A team of UAVs delivers small packages from a pile at a source point to destination points, carrying each package on a sling with a hook. One delivery is one bound unit (spec 12 §2.1): the robot that hooks a package is the one that releases it. The demo adds:

* a sling actuator and its views (§2);
* behaviours: pick up, package flight, lower and release, wait in line, recharge; normal flight is the existing fly-to (§3);
* shared zones with queues (§4);
* a delivery generator and a package manifest (§5);
* a range-limited radio and message relaying (§6);
* intermediate drops in allowed drop areas for deliveries out of loaded range (§7, *pending*).

## 2. Sling

### 2.1 Mechanism (JB, 2026-11-04)

A line of fixed length hangs below the robot with a passive hook at its end. A package carries a loop on its top. The hook catches the loop by itself when it meets it, and opens on command to let the package go. The only change of length is the hook opening: the line is `length` closed and `length_open` open. A later version may make the sling a composite node of an actuator (the hook), a sensor (a load cell) and a storage (has package / free); version 0.1 already splits it that way.

### 2.2 Device

`sling.<platform>` is a complex device (spec 04 §3) that expands to three nodes:

| Node | Class | Port key (default address) | Provides |
|---|---|---|---|
| `hook` | `SlingHook` (actuator) | `hook` (`"hook"`) | `A_REL package`: opens the hook if it holds that package (the id read by `tag`); `FAILED` for another package, none, or id 0. Closed is its safe state |
| `load` | `SlingLoad` (sensor) | `load` (`"load cell"`) | `V_SLG` |
| `tag` | `SlingTag` (sensor) | `tag` (`"tag reader"`) | `V_PAY` (the package id, 0 for none) and, while one is on the hook, `V_TAG` |

The robot file is [`examples/mavic_sling_webots.mrsd`](examples/mavic_sling_webots.mrsd) with its port map [`mavic_sling_webots.mrsp`](examples/mavic_sling_webots.mrsp): the Mavic of `mavic_webots.mrsd` plus `D_7: D_X sling sling.webots 3 hook "hook" load "load cell" tag "tag reader"`.

### 2.3 Views

| Code | Slots | Meaning |
|---|---|---|
| `V_TAG` | `num package`, `num x`, `num y`, `num z`, `num area`, `num mass` | The label of the package on the hook: its id, destination (ENU), destination area number and mass (kg) |
| `V_SLG` | `num load`, `num length`, `num open` | Load on the line (kg; 0 when the package stands on something), the line length (m) and whether the hook is open (1) |

### 2.4 Test simulator

`Sim::PackageWorld` holds every package of a run: id, bottom-centre position, tag (destination, area, mass), height (0.25 m), state (pile, carried, placed), carrier, pick and place times, and the height it fell when released. Piles are stacks at source points; only the top package of a stack can be caught. A robot's hook is `length` below it (`length_open` for `hook_open_s` = 1 s after opening). Each step:

* with nothing on the hook and the hook closed, the hook catches a pile package whose loop (top + `loop_height` 0.10 m) is within `capture_xy` 0.35 m horizontally and `capture_z` 0.15 m vertically;
* a carried package hangs below the hook; when its bottom reaches the ground or another package it stands there and the line goes slack. Its mass is added to the robot's mass while it hangs;
* opening the hook places the package where it is (it falls to the support below; the fall is recorded as `drop_height`). A placed package is never caught again.

`QuadSim` serves the ports `hook` (read: held, open; write 0: open), `load cell` (load, length, open) and `tag reader` (id, destination, area, mass, or 0). `TeamConfig::packages` shares one world between the robots; `TeamConfig::sling_length` sets the line.

## 3. Behaviours

### 3.1 Worldview

`SlingProcessor` (after `PayloadProcessor` in the chain; config `UavWorldviewConfig::delivery`) writes:

| Field | Kind | From |
|---|---|---|
| `payload.dest`, `payload.area`, `payload.mass` | vec3, scalars | `V_TAG` |
| `sling.load`, `sling.length`, `sling.open` | scalars, predicate | `V_SLG` |
| `payload.held` | predicate | `payload.id` ≠ 0 |
| `payload.grounded` | predicate | held and `sling.load` < `grounded_ratio` (0.3) × `payload.mass` |
| `payload.at_dest` | predicate | held, horizontally within `dest_tol` (1 m) of `payload.dest`, and within 1 m of the approach altitude |
| `payload.goal` | vec3 | where the package flight aims: the via point, then the destination at approach altitude `dest.z + sling.length + approach (2.5 m) + 0.5` |

**Randomised path.** When a new package is read, the processor puts a via point at the middle of the line from the pickup point to the destination, moved sideways by `u · via_offset` (8 m), where `u` in [−1, 1] is a hash of the package id (so a replay flies the same path). The via point counts as passed within `via_radius` (4 m) or once the robot is closer to the destination than the via point is. Short trips (under 2 × `via_radius`) skip it.

### 3.2 Library entries

All in [`examples/uav_behaviours.mrsb`](examples/uav_behaviours.mrsb):

| Entry | Fulfils | What it does |
|---|---|---|
| `pickup` | `C_? payload.held` | Airborne; descends at 0.3 m/s where it is until the hook holds a package. Fails after 30 s (`R_C NOT C_W 30`) |
| `flyto.package` | `C_? payload.at_dest` | Package flight: towards `payload.goal` at the robot's layer (`pkg_x`, `pkg_y`, `pkg_z`, `yaw_to_goal`), the setpoint at most 5.5 m ahead (about 5 m/s with `kp_pos` 0.9), descending to the approach altitude within 3 m. Fails if the package is lost (`R_C payload.held`) |
| `lower_package` | `C_? payload.grounded` | Descends at 0.4 m/s until the line goes slack. Fails after 40 s or if the package is lost |
| `wait_in_line` | `C_? zone._.mine` | §4.3 |
| `flyto.layered` and the other fly-to entries | `C_P3` | Normal flight (unchanged) |
| `return_home`, `land` | `C_? home`, `C_? landed` | Unchanged |

The release itself is a leaf of the delivery unit (§5.1), not a library entry: `A_FN A_REL F_L 1 1 0 payload.id`, so the robot releases whichever package it read.

### 3.3 Functions

| Key | Coefficients | Value |
|---|---|---|
| `pkg_x`, `pkg_y` | `alt_tol`, `switch_radius`, `lead` | Like `layered_x/y` towards `payload.goal`, but at most `lead` m ahead of the robot |
| `pkg_z` | `switch_radius` | `layer.alt` while farther than `switch_radius` from `payload.goal`, then `payload.goal.z` |
| `yaw_to_goal` | — | Heading to `payload.goal`; the current yaw within 1 m |
| `zone_hold_x`, `_y`, `_z` | `alt_tol`, `switch_radius` | The robot's place in the holding stack of zone `target.arg` (`zone.<id>.hold`), reached like `layered_*` |

### 3.4 Library qualifiers with a wildcard

A `C_?` entry's qualifier may use `_` for one dot-separated segment: `zone._.mine` matches `zone.3.mine` and `zone.dest.mine`. Binding a predicate (spec 03 §7) also writes `target.arg`: the number in the predicate's name, or else the field `<name without its last segment>.id` (`zone.dest.mine` reads `zone.dest.id`).

### 3.5 Recharge

`ControllerConfig::recharge` (default off). On `BATTERY_LOW` the controller blocks new tasks as before (spec 08 §4). Where it would fly home for a swap, it pushes `safety.recharge` instead, when the MRS layer names a pad (`zone.pad.id`, `zone.pad.x`, `zone.pad.y`):

1. wait for the pad (`C_? zone.pad.mine`, through `wait_in_line`);
2. fly to `pad_approach` (4 m) above it (`C_P3`, tolerance 0.3 m);
3. land (`C_? landed`);
4. charge: a `T_B` that waits until `battery.remaining` ≥ `recharge_full` (0.95).

When it succeeds the controller unblocks and journals `J_E recharged`; a task it preempted resumes. If it fails, or no pad is known, the robot swaps as before. The MRS layer picks the pad: the one this robot already wants, else the one with the shortest line, then the nearest. The pad stays wanted while the recharge runs, and is freed when the robot leaves it. In QuadSim a robot on the ground within `pad_radius` (0.6 m) of a pad charges at `charge_w` (200 W by default).

## 4. Zones and queues

### 4.1 Zones

A **zone** is a place only `capacity` robots may use at once: the pickup spot of a pile, a destination area, a charging pad. `MrsConfig::zones` lists them (`ZoneSpec`: id, centre, radius 3 m, capacity 1, holding point, pad flag); the generator writes them for its job (§5). Numbering used by the generator: source *s* is zone *s*, destination area *a* is `dest_zone_base` (200) + *a*, pads from 101. *Later:* zones in the mission file next to the header.

### 4.2 Claims

| Code | Slots after the envelope | Mode | Meaning |
|---|---|---|---|
| `M_ZONE` | `int zone`, `id op` (`want` or `free`), `num stamp` | broadcast | The sender wants the zone since `stamp`, or no longer wants it |

A robot sends `want` when it first asks for the zone and repeats it every `zone_renew` (2 s); a want not heard for `zone_timeout` (10 s) lapses. Robots line up by the stamp of their want, then by robot number; the first `capacity` hold the zone. A robot frees a zone when it has been inside the zone's radius while holding it and has moved more than 1 m beyond it, or when it has no unit any more (and no recharge running).

`A_ZQ zone` (`ZONE_QUEUE`, `I64`) asks for a zone. The executor hands it to the MRS layer (`TaskExecutor::SetZone`) and moves on; a robot with a radio has it in its profile, like `A_SY`.

### 4.3 Worldview and waiting in line

Each tick, before the robot controller, the layer writes for every zone: `zone.<id>.mine` (predicate), `zone.<id>.place` (0 first in line, −1 when not wanted) and `zone.<id>.hold` (the holding point, at `hold_base` 8 m + `hold_step` 3 m × the robot's place behind those holding the zone). For the carried package it writes `zone.dest.id` (= `dest_zone_base` + `payload.area`) and `zone.dest.mine` (TRUE when that area has no zone). For charging see §3.5. The capability profile of a robot with a radio includes `zone` (spec 04 §7).

`wait_in_line` fulfils `C_? zone._.mine`: it sends `A_ZQ target.arg`, then holds a position at the zone's holding point at the robot's place in the stack (`zone_hold_*`), until the zone is the robot's. A zone that is free ends it at the next tick.

## 5. Generator and manifest

### 5.1 The delivery unit

`Generators::DeliveryTask(id, source, top_loop, options, source_zone)` writes one `T_S` whose leaves are bound step by step (`R_K` to the leaf before, spec 12 §2.1), so one robot runs it all:

| Leaf | Start condition | Requirements | Actions, end |
|---|---|---|---|
| (zones) wait for the pile | `C_? zone.<s>.mine T` | `R_C payload.held F` | `A_N` |
| fly above the pile | `C_P3` at `top_loop + sling_length + approach` (1 m) above the ground, tolerance 0.25 m / 0.5 m | `R_C payload.held F` | `A_N` |
| pick up | `C_? payload.held T` | | end `payload.held T` |
| (zones) wait for the destination | `C_? zone.dest.mine T` | `R_C payload.held T` | `A_N` |
| package flight | `C_? payload.at_dest T` | `R_C payload.held T` | `A_N` |
| lower and release | `C_? payload.grounded T` | | `A_FN A_REL (payload.id)`, `A_W 1`; end `payload.held F` |
| return | `C_? home T` | | `A_N` |

The unit says "deliver the next package from source *s*": the destination comes from the tag (JB, 2026-11-04), so the unit does not name a package.

### 5.2 A job

`Generators::DeliveryTasks(n, sources, destinations, options)` makes `n` packages (ids from 1001), split over the sources in turn; each gets a destination drawn from the list (seeded) and a mass in [`mass_min`, `mass_max`] (0.2–0.5 kg). It returns the manifest (pile order per source, bottom first), one unit per package, and with `options.zones` the zones: one per source (holding point `hold_offset` 6 m away from the mean destination) and one per destination (holding point 6 m towards the first source). `ManifestCsv` writes `package,source,area,dest_x,dest_y,dest_z,mass` (source and area 1-based).


## 6. Radio range and relaying

### 6.1 Range in the test simulator

`Sim::Air::range_m` (`TeamConfig::radio_range`, m; default 0 = everyone hears everyone). Above 0, a message reaches only the agents within `range_m` (3D distance) of its sender at the moment it goes on the air. The operator's radio stands at `Air::operator_pos` (`TeamConfig::operator_pos`, ENU). Each reception lost to range is counted in `Air::out_of_range`. Range applies after the bitrate queue (spec 13 §3): a queued message is checked against the positions when it is sent, not when it was queued.

### 6.2 Relaying

`MrsConfig::relay` (default off) turns on `Messenger::SetRelay`. A relaying agent sends on, once and unchanged, every message it hears that:

* parses and has a known MAJOR version;
* is not its own;
* is not a unicast to itself;

whether or not it is for this agent's mission, and whether or not the agent has seen it before through its own receive path. Each `(sender, seq)` pair is sent on at most once per agent, so a flood over any graph ends after every agent has sent each message once. A relayed message keeps its envelope (sender, recipient, seq, stamp), so the receivers' duplicate rule (spec 06 §2) drops second copies and the recipient of a unicast sees it as sent by the original sender. `MessengerStats::relayed` counts the messages sent on.

Every robot relays when the option is on; there are no designated relay robots in the message layer. Consequences:

* the channel load grows with the number of relaying agents (each message is sent up to once per agent). On a limited bitrate (spec 13 §3) this is the cost of the chain;
* the spec 15 rule that sync is "never relayed" is about the content (a robot does not re-publish fields it learned from peers in its own `M_SYNC`); relaying the original `M_SYNC` message unchanged is allowed;
* the issuer does not relay.

### 6.3 Relay positions (*pending* Q5)

When a delivery's destination is beyond the issuer's range and no chain of robots covers the gap, the generator adds hold-position tasks at points along the line, spaced below `range_m`, for idle robots. Details follow the answer to Q5.

## 7. Intermediate drop points (*pending*)

## 8. Tests

* Generator: five bound leaves per unit (seven with zones), the point above a pile of three, the manifest, the same job for the same seed.
* Sling in QuadSim: only the top loop is caught; the load goes slack on the ground and the mass hangs again in the air; opening places the package; a placed package is not caught again.
* Qualifiers: `zone._.mine` matches one segment only.
* One robot delivers three packages from one pile to two destinations read at pickup: all units done, every package within 1.5 m of its destination, released less than 0.3 m above the ground (makespan about 190 s).
* Three robots share one pile and deliver nine packages to three destinations with zones on: all done, every robot carries some, no two robots below 7 m within 2 m of each other (makespan about 420 s).
* A robot that starts at 34 % battery recharges at a pad (3 kW charger in the test) and delivers all four packages without a swap.

* Relay (`tests/delivery/test_relay.cpp`): on a three-agent chain, a broadcast crosses once per agent; hearing it again sends nothing; a unicast passes through a relay that does not take it; a unicast to the relay is not sent on.
* Range: three QuadSim robots 25 m apart, the operator 20 m before the first, range 30 m. Without relay only the first robot learns the mission within 5 s; with relay all three do, and the first two have relayed messages.
