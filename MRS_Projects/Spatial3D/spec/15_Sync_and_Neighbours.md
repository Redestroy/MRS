# 15 Worldview sync, relative positions and repulsion

Status: **WP11**, spec version `0.2` (0.2 adds the sync behaviour and the capability guard, §2.5). Plan reference: §6.5 (optional worldview content), §6.6 (worldview synchronisation), §8.6 (communication), §11 (WP11). Builds on specs [04](04_Devices_and_Ports.md) (devices), [05](05_Worldview.md) (worldview), [06](06_Messages.md) (messages), [08](08_Robot_and_Flight.md) (safety supervisor) and [09](09_MRS_Layer.md) (MRS layer).

## 1. Scope

WP11 is the optional sensing package. JB's minimum (2026-10-09), in order of value:

1. **worldview sync**: robots share what they know about the world, not only their own state (§2);
2. a **relative position sensor** between robots and its processor (§3);
3. a **repulsion processor**: a force away from nearby robots, which above a certain altitude are the likeliest obstacle (§4).

Object detection beyond what exists, maps and a learned size estimate (plan §11 WP11) are not in version 0.1. Using the repulsion to fly is reactive avoidance (WP12); version 0.1 adds only an opt-in hook in the safety supervisor, to measure it (§4.3).

## 2. Worldview sync

### 2.1 What is shared

Each robot already broadcasts its own state in `M_STATE` (spec 06 §4). Sync shares **world** fields: things about the world that one robot measured and others could use. Self-state fields (`pose`, `vel`, `acc`, `att`, `rate`, `alt`, `heading`, `battery`, `geo`, the predicates, `home`, `layer`, `geofence`, `peer`, `rel`, `sync`, `repulse`, `time`) are never synced.

The **sync list** (`MrsConfig::sync`) names what a robot publishes: a list of `{prefix, period}`. Default: `{"det.", 1 s}`, the detections. A field matches an entry when its path starts with the prefix. Other world fields are added the same way (for example `{"wind.", 5 s}` for a wind estimate).

### 2.2 Views and messages

| Code | Slots after `stamp` | Meaning |
|---|---|---|
| `V_FLD` | `id path`, `num value` | One scalar world field (a vector field is sent as one view per component) |

| Message | Slots after the envelope | Mode | Meaning |
|---|---|---|---|
| `M_SYNC` | `V+` (`V_FLD` and `V_DET` views) | broadcast | World fields and detections the sender measured itself |

`M_INFOREQ` and `M_INFO` (spec 06 §4) are the P2P form: a topic that is a field prefix (`det.`) asks for every field and detection under it, and `M_INFO` answers with `V_FLD` and `V_DET` views.

### 2.3 Sending

At each entry's period, the MRS layer sends one `M_SYNC` with:

* every fresh field matching the entry whose **source is the robot's own** (not `peer:…`, §2.4) and whose stamp is newer than when it was last sent, as `V_FLD`;
* for the `det.` entry, every detection object (spec 05 §5.4) whose source is the robot's own and whose stamp is newer than when it was last sent, as `V_DET` (`class` is the path part after `det.`, without the number), within `sync_range` of the robot (0, the default: no limit).

Nothing is sent when nothing changed, so a robot with no detections sends no `M_SYNC`. A message is split to fit the transport's `MaxPayload`. On a limited channel (spec 13 §3), sync uses at most `sync_share` (0.25) of the robot's share of the bitrate; the period stretches to fit.

**Ask a peer** (the thesis's `ExchangeInfo` dialog). The first time a robot hears a peer, it sends that peer one `M_INFOREQ` with the prefixes of its sync list. The peer answers with one `M_INFO` (split to fit) holding every fresh field and detection under them that it holds, whatever their source; it sends nothing when it holds none. A robot that joins late or restarts after a battery swap so learns what the team found before.

### 2.4 Receiving

The MRS layer turns every received `V_FLD` and `V_DET` (from `M_SYNC` or `M_INFO`) into a view with source `peer:rN`, `rN` being the sender, and injects it like a peer state (spec 09 §2.3). A robot with a communication device therefore produces `V_FLD` and `V_DET` as well as `V_PEER` (amends spec 04 §7), so its self model counts detections as available even without a camera: it can learn them from peers.

* `V_DET` goes to `DetectionProcessor`, which merges it with its own detections exactly as a camera's (within 2 m of one of the same class), and writes the source `peer:rN` instead of `detector`.
* `V_FLD` goes to the new **`WorldviewSyncProcessor`**. It writes `sync.rN.<path>`, stamped with the view's stamp. When the robot's own chain neither provides nor offers `<path>` (or a prefix of it), it also **offers** `<path>` with source `peer:rN` (spec 05 §5.1), so a robot without the sensor gets the field from the freshest peer, and a robot with it uses its own.

Sources named `peer:…` rank after every other source of a field, whatever the source order (amends spec 05 §5.1). A field that came from a peer is never sent on, so values do not circle the team.

### 2.5 Asking when a condition needs it, without assuming

JB (2026-10-10): a robot that can learn a view from its peers should, when a condition on it is not TRUE, try to sync before giving up; and this must never turn into "some other robot probably has this, so it is doable".

**Which conditions.** A condition's **shared topics** are the fields it reads (spec 03 §5) that are world fields, not the robot's own state or the mission (the list of §2.1 plus `target` and `task`). A `C_V V_DET` reads `det.<class>`; a `C_m` reads its field; a `C_L` its children's. A condition with no shared topics (a pose, a predicate, the battery, a peer's state) has nothing to ask for.

**The sync behaviour.** Library entries with the qualifier `sync` (spec 03 §7) for `C_V`, `C_m` and `C_L`, priority 50, so they are tried before any entry that does the work itself:

```
B: B_E sync.view C_V sync 50 T_1/
T_1: T_B 0 1 0 C_1 C_2 C_3 T_1/
C_1: C_N/
C_2: C_N/
C_3: C_N/
T_1: T_A 0 1 0 C_1 C_2 A_1..2/
C_1: C_N/
C_2: C_N/
A_1: A_SY 2/
A_2: A_N/
```

The executor pushes it when a start condition with shared topics is FALSE (as any library behaviour) or UNKNOWN (spec 03 §8.1). Its `until` is the unmet condition. `A_SY timeout` (spec 02) is handled by the executor: on its first tick it hands the condition's shared topics to the MRS layer, which sends one `M_INFOREQ` with them to every live peer robot (§2.3); then it waits. Peers answer with `M_INFO` as in §2.3, and the answers come in with source `peer:rN` (§2.4).

**What it can and cannot conclude.**

* The condition becomes TRUE only by being evaluated TRUE on what actually arrived; the behaviour then ends at once (its `until`) and the task starts. A fact from a peer is still subject to its `max_age`.
* No peer to ask, or no answer that makes the condition TRUE within the timeout (2 s): the behaviour fails with `NOT_KNOWN` (spec 03 §8.7). The executor takes the next library entry for the condition, if there is one the robot can run; otherwise the task fails with `NOT_KNOWN`, which is not categorical, so the task goes back to the pool for a robot that knows or can find out. Each sync entry is tried once per start condition. For an UNKNOWN condition the `start_unknown_timeout` still runs from the first UNKNOWN, so asking never extends it.
* Silence is never read as "a peer has it". `A_SY` needs the messages capability (the self model adds it with any `K_M`), so a robot without a radio has no sync entry and behaves as before.

**The capability guard.** Knowing a field from peers counts for the conditions a task **reads**: a robot with a radio may take a task that starts once a person was detected, because it can learn the detection. It does not count for what a task must **make true**: the fields of its end condition (and of a `T_B`'s `until`) must be **measured** by the robot's own devices (spec 03 §5.1, spec 04 §7). A radio-only Mavic therefore cannot take a search whose end condition is a detection; a Mavic with a camera can. The same check filters library entries, so a future "search until found" behaviour for `C_V V_DET` will be chosen only by robots that can detect.

## 3. Relative positions

### 3.1 The sensor

`ranging.webots` (`D_S`, parameters `device` = `"ranging"`, `range_m`, `noise_m`) is a relative position sensor: UWB with angle of arrival, a radio direction finder, or vision. Each sample is a list of peers in range with their position relative to the robot in the ENU frame; it produces one `V_REL3` view per peer (spec 05 §6: `int robot`, `num dx`, `num dy`, `num dz`). It declares the capability `K_V V_REL3`.

| Platform | How |
|---|---|
| QuadSim | The team gives each robot its peers' true positions. A peer within `range_m` is reported with uniform noise of `noise_m` per axis, at `rate_hz` |
| Webots | From the radio receiver: each packet's emitter direction (receiver frame) and signal strength, which Webots makes `1 / r²`, so `r = 1/√s`. The direction is turned into ENU with the inertial unit's roll, pitch and yaw. The sender's robot id is read from the message envelope. The last packet per peer within `1 / rate_hz` is reported |
| ArduPilot | Not in version 0.1 (a UWB driver would be a new port) |

### 3.2 `RelativePositionProcessor`

Subscribes to `V_REL3`, provides `rel` (the `rel.<id>.*` subtree). A `V_REL3` from peer `N` writes `rel.rN.enu` (vec3) and `rel.rN.range` (scalar), stamped with the view's stamp. `rel` fields have a `max_age` of 1 s.

### 3.3 Neighbours

`Environment::Neighbours(w, t)` lists every peer the worldview knows, with its position relative to the robot, best first:

* `rel.rN.enu` when fresh (`measured = true`): a direct measurement, independent of both robots' GNSS errors;
* otherwise `peer.rN.pose.enu − pose.enu` when both are fresh (`max_age` 2 s for peer states).

The relative velocity is `peer.rN.vel.enu − vel.enu` when both are fresh, else 0. The repulsion processor (§4) uses this list.

## 4. Repulsion

### 4.1 `RepulsionProcessor`

No subscriptions and no needs, so it is in every chain; it provides `repulse` (`repulse.enu`, `repulse.nearest`, `repulse.count`). In every tick, for each neighbour at relative position `d` with relative velocity `u`:

* look ahead: `d' = d + u·τ`, with `τ` the time of closest approach clamped to `[0, lookahead]` (1 s), so robots closing in react earlier;
* the normalised ellipsoidal distance `s = √((d'x² + d'y²)/R² + d'z²/H²)`, with `R = radius_xy` (4 m) and `H = radius_z` (2 m). The vertical radius is smaller because robots fly in altitude layers 5 m apart (spec 09), which should not push each other;
* when `s < 1`, the force has magnitude `min(gain · (1/s − 1), max_speed)` (gain 0.5 m/s, max 3 m/s), directed away from the neighbour along the ellipsoid's gradient `(−d'x/R², −d'y/R², −d'z/H²)`, normalised. Two robots at exactly the same point both push upwards.

`repulse.enu` is the sum (m/s, ENU), `repulse.nearest` the Euclidean distance to the nearest neighbour (missing with none) and `repulse.count` the number of neighbours with `s < 1`. With no neighbours the force is `(0, 0, 0)`.

### 4.2 Defaults

| Parameter (`UavWorldviewConfig`) | Default |
|---|---|
| `repulse_radius_xy` | 4 m |
| `repulse_radius_z` | 2 m |
| `repulse_gain` | 0.5 m/s |
| `repulse_max` | 3 m/s |
| `repulse_lookahead` | 1 s |

### 4.3 Opt-in use in the safety supervisor

`SafetyConfig::repulsion_lead` (s, default 0: off). When it is above 0 and the robot is airborne, the supervisor adds `repulse.enu` to velocity setpoints (`A_VXY`, `A_VZY`) and `repulsion_lead · repulse.enu` to position setpoints (`A_PXY`, `A_PZY`), before its fence and speed limits. Tasks and allocators do not change. This is for measuring the force (§5); WP12 replaces it with a proper avoidance strategy.

## 5. Evaluation

QuadSim, `mrs_experiment run ... --repulsion LEAD,.. [--ranging NOISE_M]` (every run flown at each lead; `--ranging` gives every robot the sensor with that noise; the CSV gains `repulsion_lead_s` and `ranging_noise_m`): the 30-task flat sets at N = 8 with open and exclusive MRS-RTA, where WP6/WP7 found most separation breaches, with the repulsion off and on. Reported: separation breaches, minimum separation, makespan and stalled runs. Results in `experiments/uav_spatial/results/wp11`.

**Results** (500 runs, 50 sets × 2 allocators × leads 0, 1, 2 s, plus leads 1 and 2 with the ranging sensor; all completed). Breaches per run at N = 8:

| allocator | off | lead 1 s | lead 2 s |
|---|---|---|---|
| G-RTA (open) | 12.9 | 8.1 | 3.0 |
| G-RTA-X (exclusive) | 2.1 | 1.3 | 1.1 |

Makespan changes by less than 1%. The breaches left under open MRS-RTA are most likely robots sent to the same target (inferred from the WP6 traces), which the push slows but cannot keep apart. With the ranging sensor (0.1 m noise) the numbers are the same as from peer states, since QuadSim's GPS is good.

## 6. Tests (WP11)

* `V_FLD` and `M_SYNC` parse, round-trip and reject other views inside `M_SYNC`;
* `WorldviewSyncProcessor`: a peer field lands in `sync.rN.<path>` and is offered for `<path>`; the robot's own source wins over it; a field the robot provides is not overwritten;
* two robots on QuadSim: a detection injected into r1 reaches r2's worldview with source `peer:r1` and is not sent back; a robot started later learns it through `M_INFOREQ`;
* the ranging sensor reports peers within range only, with the right sign; `rel.rN.enu` is written;
* `Neighbours` prefers a fresh relative measurement over peer states;
* repulsion: zero without neighbours, pointing away, stronger when closer, zero beyond the radius, small for a peer one layer up, and the supervisor hook moves a setpoint only when enabled.
* the sync behaviour: a FALSE detection condition asks the peers for `det.person`; an answer that makes it TRUE starts the task; no answer fails it with `NOT_KNOWN` after the timeout; no peer to ask fails it at once; an UNKNOWN world field is asked for once and still fails with `MISSING_FIELD` after `start_unknown_timeout`; a condition on the robot's own state asks nothing;
* the capability guard: the radio-only Mavic provides but does not measure `det`, can take a task that starts on a detection, cannot take one that ends on a detection unless `det` is measured; without a radio it has neither the field nor a sync entry;
* two UAVs on QuadSim: r2 needs a detection only r1 holds (not broadcast); r2's sync behaviour gets it from r1 and the task succeeds; for a class nobody holds the task fails with `NOT_KNOWN` and no detection appears.
