# 09 MRS layer: messages, task pool and MRS-RTA

Status: **WP5**, spec version `0.1`. Plan reference: §8 (MRS layer), §9.1 (task issuer), §10.3 (ported task sets), §11 (WP5). Builds on specs [03](03_Tasks_and_Execution.md) (tasks), [06](06_Messages.md) (messages, dump rule, claims, mission header, timeline) and [08](08_Robot_and_Flight.md) (robot controller).

## 1. Scope

WP5 makes a team of robots share one task stream:

* the **messenger** puts spec 06 messages on a transport and filters what comes back (§2);
* the **MRS layer** of each robot keeps a task pool and a peer table, applies the dump rule, and feeds the robot controller the task its allocator picks (§3, §5);
* the **allocator seam** and **MRS-RTA**, in open and exclusive mode (§4);
* the **task issuer** sends the mission and a timeline, and records results (§6);
* the **2021 task sets** are ported to timelines (§7);
* Webots controllers for the UAVs and the issuer (§8).

Code: `MRS::Comm` in `core/comm/`; `MRS::Algorithms` in the new `algorithms/` library (`mrs_algorithms`, which depends on `mrs_core`).

## 2. Messages

### 2.1 Messenger

`Messenger(transport, self)` owns the sender name and the per-mission sequence number.

* `Begin(code, recipient, t)` returns an `M` record with the envelope of spec 06 §2 filled in: version 0.1, mission, sender, recipient, the next `seq`, and `stamp` rounded to the millisecond. The caller appends the type's slots.
* `Post(record)` writes the message form (one line, canonical) and sends it. A message larger than the transport's `MaxPayload` is not sent and is counted.
* `Receive()` returns every new message for this agent and drops, with a counter each: text that does not parse as one `M` record, another MAJOR version, the agent's own messages, unicasts to someone else, messages of another mission, and repeated `(sender, seq)` pairs. Until a mission is set, only `M_MISSION` passes, together with the messages of its mission that follow it in the same poll (tasks the issuer sends in its first tick; found in WP6).

### 2.2 Transports

`ITransport` has `Send(text, recipient)`, `Poll()` and `MaxPayload()`. A broadcast-only link sends a unicast as a broadcast; the envelope's recipient makes the others ignore it (spec 06 §2).

* `CommBlockTransport` sends and receives through the robot's communication devices. On Webots that is the `radio` node, an Emitter and a Receiver on one channel, so it is the Webots radio transport of the plan.
* The issuer in Webots uses the supervisor's own Emitter and Receiver (§8).
* The tests use an in-memory channel that delivers every message to everyone else at the next tick.

### 2.3 Peer state

Every robot broadcasts `M_STATE` at 2 Hz once it has a position: a `V_PEER` view with its number, `pose.enu`, `vel.enu`, `battery.remaining` and the task it pursues (`0` for none). A received `M_STATE` updates the peer table and is passed to the robot controller as a view (`RobotController::InjectViews`), so `PeerStateProcessor` writes `peer.<name>.*` in the worldview. `M_PROFILE` (the self model, spec 06 §4) goes out at the first tick with a mission and every 10 s, so robots that start late learn it.

## 3. Task pool and peer table

`TaskPool` holds every task the robot has heard of, keyed by task id (never by position, which fixes the 2021 `TaskPriorityList` overwrite bug):

| Field | Meaning |
|---|---|
| `task` | the task as received in `M_TASK` |
| `state` | a pool state of spec 06 §8: `AVAILABLE`, `ACTIVE` (this robot pursues it), `DONE`, `FAILED`, `DUMPED_SELF`, `IMPOSSIBLE`, `CANCELLED` (`BLOCKED` and `CLAIMED` are reserved for WP8; claims are kept per entry instead) |
| `arrival` | when this robot first heard of it (`Δt` of §4.2) |
| `claims` | live claims by robot: owner, expiry, score; expired ones are dropped every tick |
| `static_dumps` | robots that sent `M_DUMP` with `static = T`, this one included |
| `retry_after` | the dynamic-dump cooldown on this robot |

A known id received again changes nothing (spec 06 §4). `PeerTable` holds, per robot heard by `M_STATE` or `M_PROFILE`: when it was last heard, its pose, velocity, battery, current task, and its profile (type, roles, actions). A peer is alive when heard within `peer_timeout` (10 s).

## 4. Allocation

### 4.1 The seam

```cpp
class IAllocator {
  virtual void Bind(const AllocatorContext&);          // pool, peers, self, profile, size estimator, resources, claim/release
  virtual void OnTaskReceived(const std::string& task, double t);
  virtual void OnMessage(const Comm::Message&, double t);
  virtual void OnTaskFinished(const std::string& task, PoolState, double t);
  virtual void OnCapabilityChanged(double t);
  virtual Decision Select(const Worldview&, const CurrentTask&, double t) = 0;   // KEEP, SWITCH(task), IDLE
  virtual AllocatorInfo Info() const = 0;                                         // name, exclusive?, uses comms?
};
```

Allocators read the worldview, the pool and the peers, and send only claims and releases (through the context). They never touch devices. `CurrentTask` names the task the robot pursues and whether its actions have started (`IN_PROGRESS`; for a complex unit, once its first leaf runs, spec 12 §2.2). A task is **eligible** when it is not finished, not `DUMPED_SELF`, not `BLOCKED` (a gated tree leaf, spec 11 §2.3), past its retry cooldown, and none of its runtime conditions is FALSE now (spec 11 §2.4a, WP7).

### 4.2 Priority

`priority = base_priority × size_est(task, w, t)` (JB decision 2), with the base priority applied once. The default `SpatialSizeEstimator` (plan §8.4):

```
size_est = a1·Δt + a2 / (Δs3 + ε) − a3·energy_fraction − a4·claim_penalty, at least 0.001
```

| Term | Value | Default |
|---|---|---|
| `Δt` | time since this robot heard of the task | `a1` = 0.001 per s |
| `Δs3` | travel to the task's first `C_P3` the layered way: `|layer.alt − z| + horizontal + |layer.alt − target.z|`, or straight without a layer or when the horizontal distance is at most the fly-to's switch radius (3 m; WP7: a robot over its target was scored as if it had to climb to its layer first, and once idle robots waited at their layers, exclusive claims flipped between robots without end); 0 for a task without a position | `a2` = 10 m, `ε` = 1 m |
| `energy_fraction` | `ResourceManager::TaskCost` / `battery.energy_wh` (spec 08 §6), at most 1 | `a3` = 0.5 |
| `claim_penalty` | 1 when a peer that claims the task is closer to it | `a4` = 0.5 |

The result is multiplied by `pursuit` (0.2) when a live peer that is closer to the task reports it as its current task in `M_STATE`. Without this, open mode sends every robot to the same task: in the five-UAV test (§9) one robot did 11 of 15 tasks and three did none, with a makespan of 194 s; with it, every robot works and the makespan is 101 s. `pursuit = 1` gives the 2021 behaviour.

### 4.3 MRS-RTA

Every tick, `RtaAllocator::Select` takes the eligible task of highest priority.

* **Keep or switch.** While the current task's actions run (`started`), the robot keeps it. Before that, it switches only to a task whose priority is more than `switch_margin` (25 %) above the current one, so robots do not flip between two similar tasks.
* **Done by someone else.** When `M_DONE` (or `M_FAIL`, or `M_CMD cancel`) arrives for the robot's task, the MRS layer takes the task back from the robot controller (§5) and the next tick picks another.
* **Open mode** (default) ignores claims; several robots may pursue one task until one finishes it.
* **Exclusive mode** (`exclusive = T`, `MRS-RTA-X`) claims the task it selects (`M_CLAIM` with `expiry = t + claim_ttl`, 10 s, and `score` = its priority), renews the claim at half its life, and releases it when it switches or idles. A task that another robot claims with a stronger claim (spec 06 §4.1: the higher score, then the lower robot number) is not eligible. A robot whose own task is outclaimed drops it at once, unless its actions have started (WP8, spec 12 §3.2: a started task keeps its claim, and the claim's score does not drop while the robot works on it; before WP8 a started task was dropped too, which cost a robot a half-flown complex unit once it moved away from the unit's first target). For atomic tasks the change is not measurable: a robot whose actions run is at the target and already has the top score (45 WP6 runs re-flown: 40 identical, 5 at N = 8 within ±2.5%, condition means within 0.5%).

## 5. The MRS layer

`MrsLayer(controller, transport, registry, allocator)` runs one robot's MRS side. `Tick(t)`:

1. Receive and handle messages (§5.1).
2. Expire claims.
3. With a mission: `Select`, and apply the decision. `SWITCH` takes the previous task back from the controller (`RobotController::CancelTask`) and adds the new one (`AddTask`); `IDLE` takes the current task back. No allocation while the controller blocks new tasks (battery, spec 08 §4) or has stopped.
4. `controller.Tick(t)`.
5. Handle the controller's task and robot events (§5.2).
6. Send `M_PROFILE` and `M_STATE` when due.

`RobotController::CancelTask(id)` removes a list task from the list or from the executor's stack (`TaskExecutor::Withdraw`, which also clears `target.*`), without a task event, and journals it as `AVAILABLE`. It refuses while a safety behaviour (a return home or a landing) runs above the task, so the layer tries again at the next tick. With an MRS layer, the controller does not journal `J_E end` when its list runs empty.

### 5.1 Incoming

| Message | Handling |
|---|---|
| `M_MISSION` | The first one sets the controller's mission and the messenger's; later ones are ignored. `claim_grace` comes from the header |
| `M_TASK` | Each new task enters the pool. A task that fails the static checks (spec 03 §5) against the robot's profile is `DUMPED_SELF` at once and `M_DUMP … IMPOSSIBLE T 0` is sent. A task the robot cannot load is ignored |
| `M_DONE` | `DONE`, with who did it; taken back from the controller if it is the robot's task |
| `M_FAIL` | `FAILED`, or `IMPOSSIBLE` when that is the reason; taken back likewise |
| `M_DUMP` | The sender's claim is dropped. With `static = T` the sender joins `static_dumps`, and the impossible check runs (§5.3) |
| `M_CLAIM`, `M_RELEASE` | The sender's claim is set or dropped |
| `M_STATE`, `M_PROFILE` | §2.3 |
| `M_CMD` | `cancel` and `abort` with a task id make it `CANCELLED`. Other commands are WP9 |

### 5.2 The dump rule

The controller's task events apply spec 06 §5:

| Event | Pool state | Message |
|---|---|---|
| `SUCCEEDED` | `DONE` | `M_DONE` |
| `FAILED` with `NO_BEHAVIOUR`, `NO_ACTUATOR` or `IMPOSSIBLE` (including a target outside the fence, spec 08 §3.2) | `DUMPED_SELF` | `M_DUMP` with `static = T` and the action iterator reached |
| `FAILED` with `END_NOT_MET` or `ABORTED` | `FAILED` | `M_FAIL` |
| `FAILED` for any other reason (`MISSING_FIELD`, `RUNTIME_CONDITION`, `START_UNREACHABLE`, `PREEMPTED_OUT`, …) | `AVAILABLE`, retry after 30 s on this robot | `M_DUMP` with `static = F` |

On `BATTERY_LOW` or `BATTERY_CRITICAL` (spec 08 §4), the robot sends `M_CLAIM` for its task with `expiry = t + claim_grace` and score 0 (spec 06 §4.1), so peers in exclusive mode leave the task alone during the swap.

### 5.3 Impossible tasks

When this robot and every live peer have dumped a task statically, it is `IMPOSSIBLE`, and the robot sends `M_FAIL <task> IMPOSSIBLE`. The check runs when the robot dumps the task and whenever a peer's static dump arrives. A robot that has not been heard does not count, so a task can become impossible while a capable robot is still out of range; the issuer may dispatch it again (WP9).

## 6. Task issuer

`TaskIssuer(transport, mission_header)` is the operator's agent:

* `LoadTimeline(.mrsl)` and `Schedule(t, tasks)` put tasks on a timeline sorted by time, keeping file order for equal times (spec 06 §7).
* `Tick(t)` sends `M_MISSION` at the first tick and every `mission_period` (5 s), so robots that start late get it; sends each due task in its own `M_TASK` (spec 06 §2); optionally repeats open tasks every `task_period` (off by default); and reads `M_DONE`, `M_FAIL` and static `M_DUMP`.
* Per task it records dispatch time, first completion and by whom, the number of `M_DONE` messages (more than one means duplicate arrivals), the failure reason, and who dumped it.
* `Done()` when every scheduled task was sent and has ended; `Makespan()` is first dispatch to last completion. `Cancel(task)` sends `M_CMD cancel`.

## 7. The 2021 task sets

`Port2021TaskSet(text, config)` and the tool `mrs_port2021 <TaskSetN.dat> [altitude]` turn a 2021 E-puck task set into a timeline. A 2021 line is `<time> T: <id> <state> T_A <priority> /C_S C_P x y z tol/C_E C_N/T_A n /A_L k/A_W w/…`.

| 2021 | Ported |
|---|---|
| dispatch `time` (s) | `L_D time` |
| task `id` | `op.<id + 1>` |
| `priority` | the same base priority |
| `C_P x y z tol` | `C_P3 (x − 500)·0.1 (y − 500)·0.1 15 1 0.5 0 -1`: 0.1 m per unit, the 2021 point (500, 500) at the origin, 15 m up |
| `A_L k`, alternately | `A_L 3 <colour of k>` (on) and `A_L 3 0` (off), so the 2021 toggle is kept |
| `A_W w` | `A_W w·0.1` s |

All 25 sets are ported in [`experiments/uav_spatial/tasksets_2021/`](../experiments/uav_spatial/tasksets_2021/), with a mission header for five UAVs (`mission_5uav.mrs`). Targets fall within about ±70 m of the origin.

## 8. Webots controllers

Built with the Webots platform (spec 07 §1); checked against the Webots headers but not yet run in Webots.

* `mrs_uav <id> [rta|rta-x|sta|cbba|ldta2] [definition] [ports] [behaviours]` builds the robot from the definition with its head node id replaced by `<id>`, runs the robot controller under the MRS layer with open or exclusive MRS-RTA, MRS-STA (spec 12 §3), CBBA or LDTA² (spec 13), and appends its journal to `r<id>.mrsj`. The Mavic needs an Emitter `emitter` and a Receiver `receiver` in a body slot.
* `mrs_issuer <mission header> <timeline> [results.csv] [channel]` runs on a supervisor with its own Emitter and Receiver. It writes `task, dispatch, done, done_by, done_count, failed` per task when every task has ended, and pauses the simulation.

## 9. Tests

`tests/team/` (spec 07 §5.7):

* Messenger: the envelope and message form; dropping duplicates, other missions, unicasts to others, broken text and own messages; only `M_MISSION` before a mission.
* Claims: the higher score, then the lower robot number; expiry; a known task id changes nothing.
* The 2021 port: coordinates and waits of one line; all 25 ported sets parse; the issuer dispatches in time order.
* Five UAVs in `QuadSim` on one radio channel fly ported set 1 with open MRS-RTA: all 15 tasks done, at least four robots do work, no crash. Exclusive mode on the same set: all done, each exactly once.
* A UAV built without LEDs dumps all 15 LED tasks statically and does none; the other four do them all.
* Two UAVs without LEDs and one LED task: the task becomes `IMPOSSIBLE` on both and the issuer gets `M_FAIL IMPOSSIBLE`.
* Two UAVs and one task in open mode: after the first finishes it, neither robot still has it.
