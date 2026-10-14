# 06 Messages, task ids, mission header, timeline and journal

Status: **draft for WP0**, spec version `0.1`. Grammar: [01](01_Protocol_Grammar.md). Plan reference: §8.3, §8.6, §9 and §15 (battery swaps).

## 1. Agents and names

| Agent | Name | Notes |
|---|---|---|
| Robot with team id `N` (spec 04 §2.3) | `r<N>`, for example `r3` | `N` ≥ 1 |
| Operator / task issuer | an issuer identifier, default `op` | Several issuers MAY exist; names MUST differ |
| Everyone | `all` | Only as a recipient |

## 2. The message envelope

Every message is one top-level `M` record with its sub-records.

```
M: <code> <ver> <mission> <sender> <recipient> <seq> <stamp> <type-specific slots>/
```

| Slot | Type | Meaning |
|---|---|---|
| `ver` | `num` | Protocol version, `0.1` |
| `mission` | `id` | Mission id from the mission header. Messages of another mission are dropped |
| `sender` | `id` | Sender name (§1) |
| `recipient` | `id` | Recipient name, or `all` for broadcast |
| `seq` | `int` | Per-sender sequence number, starting at 1 for each mission. Used to drop duplicates |
| `stamp` | `num` | Mission time at sending |

Rules:
* A receiver MUST drop a message that fails to parse, has an unknown MAJOR version, or a `(sender, seq)` pair it has already seen, and count the drop.
* A unicast message to another robot is ignored by everyone else, so a broadcast-only transport can emulate P2P (plan §8.6).
* Messages MUST NOT contain comments, and SHOULD be written in canonical form (spec 01 §3.5) with records on one line, separated by nothing.
* A message MUST fit the transport's `max_payload`. A `M_TASK` that does not fit MUST be split into one message per task; a single task that does not fit is an error reported to the issuer.

## 3. Task ids

```
taskid = issuer "." seq { "." child }
```

* `issuer` is the name of the agent that created the task: `op` for operator tasks, `r3` for a task a robot generated itself.
* `seq` is unique per issuer within a mission, starting at 1.
* `child` indexes are assigned by the decomposer (spec 03 §6.2). Example: `op.17.2.1`.
* A task id is never reused within a mission.

## 4. Message types

| Code | Slots after the envelope | Mode | Meaning |
|---|---|---|---|
| `M_MISSION` | `H` | broadcast, from the issuer | The mission header (§6). Sent at start and again on request |
| `M_TASK` | `T+` | broadcast, from the issuer or relayed | New tasks. Receiving a known id again changes nothing |
| `M_CLAIM` | `tid task`, `num expiry`, `num score` | broadcast | The sender is working on `task` (exclusive mode). Valid until mission time `expiry` |
| `M_RELEASE` | `tid task` | broadcast | The sender gives up its claim |
| `M_DONE` | `tid task` | broadcast | `task` ended `SUCCEEDED` |
| `M_FAIL` | `tid task`, `id reason` | broadcast | `task` ended `FAILED` with a reason from spec 03 §8.7 that does not return it to the pool |
| `M_DUMP` | `tid task`, `id reason`, `bool static`, `int progress` | broadcast | The sender will not do `task` (§5). `progress` is the action iterator reached (0 if not started) |
| `M_STATE` | `V` (a `V_PEER` view) | broadcast, at `state_rate` (default 2 Hz) | Heartbeat: pose, velocity, battery, current task |
| `M_PROFILE` | `id robot_type`, `int n`, `n × id role`, `K*` | broadcast, on join and on change | Capability profile from the self model (spec 04 §7) |
| `M_BID` | `int n`, `n × (tid task, num bid, id winner, num time)` | broadcast | CBBA winning-bid list (task, winning bid, winning agent, time of the bid). Winner `0`: none (the sender gave the task up). Spec 13 §2.2 |
| `M_INFOREQ` | `int n`, `n × id topic` | P2P | Ask a peer for information. Topics: `mission`, `tasks`, `profile`, or a field prefix (robots answer field prefixes only in version 0.1; spec 15 §2.3) |
| `M_INFO` | any records | P2P | Answer to `M_INFOREQ`: `H`, `T`, `K` or `V` records (`V_FLD` and `V_DET` for a field prefix) |
| `M_SYNC` | `V+` (`V_FLD` and `V_DET` views) | broadcast | World fields and detections the sender measured itself (WP11, spec 15 §2) |
| `M_CMD` | `id command`, `tid task` | broadcast or P2P, from the issuer | `abort`, `recall`, `pause`, `resume`, `cancel` (with a task id; `0` for commands about the whole mission) |
| `M_PLAN` | `int revision`, `int n`, `n × tid task` | P2P, from the issuer | The recipient's route from the central planner (G-C, spec 10 §2.3), in order. A higher revision replaces the route; a lower or equal one is ignored |

Example: `r2` claims task `op.17` for 30 s.

```
M: M_CLAIM 0.1 m1 r2 all 57 142.350 op.17 172.350 0.84/
```

### 4.1 Claims (exclusive mode)

* A robot in exclusive mode sends `M_CLAIM` when it selects a task, and renews it before `expiry`.
* Two claims on one task: the higher `score` wins; on a tie, the lower robot id wins. The loser drops the task at once and sends `M_RELEASE`.
* A claim not renewed by `expiry` lapses, and the task becomes `AVAILABLE` again.
* **Battery swap (JB decision 2):** before landing for a swap, a robot sends `M_CLAIM` for its active task with `expiry = now + claim_grace` (default 120 s). Peers leave the task alone until then. After the restart, the robot reloads its journal (§8), lifts off and renews the claim.

## 5. The dump rule

A robot applies this rule whenever a task fails a check or ends `FAILED`.

| Situation | Pool state on this robot | Message |
|---|---|---|
| Static requirement fails (spec 03 §5), or the task failed with `NO_BEHAVIOUR`, `NO_ACTUATOR` or `IMPOSSIBLE` | `DUMPED_SELF`, permanent | `M_DUMP` with `static = T` |
| Dynamic requirement fails before starting | unchanged; skipped this tick | none |
| Failed with `MISSING_FIELD`, `RUNTIME_CONDITION`, `START_UNREACHABLE` or `PREEMPTED_OUT` | `AVAILABLE`, with a retry cooldown (default 30 s) on this robot | `M_DUMP` with `static = F` |
| Failed with `END_NOT_MET` or `ABORTED` | `FAILED` | `M_FAIL` |

* A peer that receives `M_DUMP` with `static = F` treats the task as `AVAILABLE`.
* When every robot in the peer table that is alive (heard within `peer_timeout`, default 10 s) has sent `M_DUMP` with `static = T` for a task, the task becomes `IMPOSSIBLE` and the issuer is told with `M_FAIL ... IMPOSSIBLE`.

## 6. Mission header

| Code | Slots |
|---|---|
| `H_M` | `id mission`, `num lat0`, `num lon0`, `num alt0`, `num xmin`, `num xmax`, `num ymin`, `num ymax`, `num zmax`, `num layer_base`, `num layer_step`, `num claim_grace`, `int n`, `n × (int robot, num x, num y, num z)` |

* `lat0 lon0 alt0` is the geo reference: the ENU origin for the whole team.
* The geofence in version 0.1 is the ENU box `[xmin, xmax] × [ymin, ymax] × [0, zmax]`.
* A robot's cruise altitude is `layer.alt = layer_base + (id − 1) × layer_step`.
* The list gives each robot's home position in ENU.
* Mission time `t = 0` is the moment the issuer first sends `M_MISSION`.

Example:

```
H: H_M m1 56.9496 24.1052 10 -500 500 -500 500 120 20 5 120 3 1 0 0 0 2 5 0 0 3 10 0 0/
```

## 7. Timeline (`.mrsl`)

A timeline file schedules tasks for the issuer.

| Code | Slots |
|---|---|
| `L_D` | `num t`, `T+` |

* `t` is the dispatch time in mission seconds. The issuer sends the tasks in one `M_TASK` (or several, §2) at mission time `t`.
* Records MAY appear in any order. The issuer MUST sort them by `t`, keeping file order for equal `t`. (The 2021 supervisor did not sort, which mis-timed 10 of the 25 task sets.)

```
@: MRS 0.1/
L: L_D 0 T_1/
T_1: T_A op.1 1 0 C_1 C_2 A_1/
C_1: C_P3 -100 -100 15 1 0.5 0 -1/
C_2: C_N/
A_1: A_N/
L: L_D 5 T_1/
T_1: T_A op.2 1 0 C_1 C_2 A_1/
C_1: C_P3 -50 -100 15 1 0.5 0 -1/
C_2: C_N/
A_1: A_N/
```

## 8. Task journal (`.mrsj`)

The journal lets a robot resume its work after a battery swap or a restart (JB decision 2). It is an append-only file of `J` records on the robot's storage device.

| Code | Slots | Written when |
|---|---|---|
| `J_H` | `num stamp`, `id robot`, `H` | First record: robot name and the mission header |
| `J_T` | `num stamp`, `id pool_state`, `int iterator`, `T` | A task's pool or execution state changes. `T` is the full task with its current execution state in the state slot; `iterator` is its action iterator |
| `J_K` | `num stamp`, `int n`, `n × tid` | The active-task stack changes; listed bottom to top |
| `J_C` | `num stamp`, `tid task`, `id owner`, `num expiry` | A claim is made, renewed, lost or released (`expiry = 0` means released) |
| `J_E` | `num stamp`, `id event` | Lifecycle events: `start`, `swap_land`, `restart`, `end` |

Rules:
* Records are appended and flushed at once. A partly written last record (no final `/`) is ignored on reload.
* On reload, the last `J_T` per task id, the last `J_K` and the last `J_C` per task id give the state.
* On restart with a journal for the current mission (same `mission` in `J_H`): restore the pool, claims and stack; tasks that were `IN_PROGRESS` or `STARTED` become `IDLE`; run `Liftoff`; then resume normally, so each resumed task's start condition is evaluated again (spec 03 §8.6).
* A journal of a different mission is archived, not loaded.
* A robot MAY rewrite the journal as a compact snapshot (one `J_H`, the current `J_T`, `J_K` and `J_C` records) when it grows beyond `journal_max_kb` (default 256).
