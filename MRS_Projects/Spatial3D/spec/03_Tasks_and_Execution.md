# 03 Tasks, conditions and execution

Status: **draft for WP0**, spec version `0.1`. Grammar: [01](01_Protocol_Grammar.md). Actions and functions: [02](02_Actions.md). Worldview fields: [05](05_Worldview.md). Task ids and pool messages: [06](06_Messages.md).

## 1. The task model

A task is the tuple from the 2021 paper, `t = <id, b_priority, state, C_start, C_end, C_runtime, A, H>`, extended with requirements and children:

| Element | In this spec |
|---|---|
| `id` | Task id, `issuer.seq[.child…]` (spec 06 §3) |
| `b_priority` | Base priority, a number > 0 |
| `state` | Execution state (§3.1) |
| `C_start`, `C_end` | Start and end conditions (§4) |
| `C_runtime` | Runtime conditions, carried by an `R_C` requirement (§5) |
| `A` | Actions (spec 02), for leaf tasks |
| `H` | The relation between runtime conditions and actions. In version 0.1, every runtime condition guards every action of the task |
| Requirements | What a robot needs to take the task (§5) |
| Children | Sub-tasks and their link type, for complex tasks (§6) |

## 2. Task codes and slots

Common slots, in this order, start every task body: `tid id`, `num priority`, `int state`.

| Code | Kind | Slots after the common ones | Notes |
|---|---|---|---|
| `T_A` | Atomic | `C start`, `C end`, `[R req]`, `A+ actions` | Actions MUST NOT include `A_FN` |
| `T_P` | Parametric atomic | `C start`, `C end`, `[R req]`, `A+ actions` | Like `T_A`; actions MAY include `A_FN` |
| `T_B` | Behaviour | `C start`, `C end`, `C until`, `[R req]`, `T base` | Repeats `base` until `until` holds (§8.4) |
| `T_S` | Sequential | `C start`, `C end`, `[R req]`, `T+ children` | Children run one after another, in order |
| `T_L` | Parallel | `int k`, `C start`, `C end`, `[R req]`, `T+ children` | Done when `k` children are done; `k = 0` means all |
| `T_O` | Choice | `C start`, `C end`, `[R req]`, `T+ children` | Done when one child is done |
| `T_D` | Defined link | reserved | Master's thesis "defined" link |
| `T_G` | Goal-state task | reserved | |
| `T_U` | Utility task | reserved | |

Rules:
* The optional `R` slot is recognised by its kind letter.
* `priority` MUST be > 0. The **effective base priority** of a task inside a tree is the product of its own priority and every ancestor's priority.
* A task file sends `state = 0` (NEW). Other values appear only in journals (spec 06 §8).

Example: the UAV equivalent of a 2021 E-puck task.

```
T: T_A op.17 1 0 C_1 C_2 R_1 A_1..4/
C_1: C_P3 120 -40 15 1 0.5 0 -1/
C_2: C_N/
R_1: R_F 0.5 pose.enu alt.agl heading/
A_1: A_L 1 65280/
A_2: A_W 2/
A_3: A_L 1 0/
A_4: A_N/
```

## 3. States

### 3.1 Execution state (in the task record)

The numbering keeps the existing `MRS::Task::TaskState` enum.

| Value | State | Meaning |
|---|---|---|
| 0 | `NEW` | Known, never started |
| 1 | `IDLE` | Started earlier and suspended by preemption (§8.6) |
| 2 | `QUEUED` | Selected by the allocator, not yet started |
| 3 | `STARTED` | Selected; its start condition is being fulfilled by a behaviour |
| 4 | `IN_PROGRESS` | Start condition met; actions running |
| 5 | `SUCCEEDED` | Ended with the end condition true |
| 6 | `FAILED` | Ended otherwise, with a reason (§8.7) |

Transitions:

```
NEW ─► QUEUED ─► STARTED ─► IN_PROGRESS ─► SUCCEEDED
          │          │            │
          │          └──────┬─────┴──────► FAILED
          │                 ▼
          └───────────────► IDLE ─► QUEUED (on resume)
```

A task that starts with its start condition already true goes from `QUEUED` straight to `IN_PROGRESS`.

### 3.2 Pool state (per robot, in the MRS layer)

Each robot keeps its own view of every task it knows (spec 06 §4):

| Pool state | Meaning |
|---|---|
| `AVAILABLE` | Can be taken |
| `BLOCKED` | A predecessor in its tree is not done (§6) |
| `CLAIMED` | Claimed by a peer (exclusive mode); the claim has an owner and an expiry |
| `ACTIVE` | This robot is executing it |
| `DONE` | Some robot reported it `SUCCEEDED` |
| `FAILED` | Ended in `FAILED` and was not returned to the pool |
| `DUMPED_SELF` | This robot can never do it (static requirement failure) |
| `IMPOSSIBLE` | Every known peer has dumped it |
| `CANCELLED` | No longer needed (its choice or parallel parent is done, or the operator cancelled it) |

## 4. Conditions

A condition evaluates to **TRUE**, **FALSE** or **UNKNOWN**. UNKNOWN means a field it reads is missing or stale (spec 05 §3). Logic uses Kleene three-valued rules.

| Code | Name | Slots | TRUE when | Fields read |
|---|---|---|---|---|
| `C_N` | Null | — | always | — |
| `C_F` | False | — | never | — |
| `C_?` | Predicate | `?` | the predicate field equals the literal's value | the predicate name |
| `C_m` | Parameter | `id field`, `id op`, `num value`, `num tol` | `w[field] op value`; `op` ∈ `lt le eq ne ge gt`; `eq` and `ne` use `tol` | `field` |
| `C_T` | Absolute time | `num t` | mission time ≥ `t` | `time` |
| `C_W` | Elapsed time | `num d` | at least `d` seconds have passed since the condition's context started (§8.3) | `time` |
| `C_L` | Logic | `id op`, `C+` | `op` ∈ `AND OR XOR NOT`; `NOT` takes exactly one child | children's fields |
| `C_V` | View match | `V` | the worldview holds a view of that type that matches (spec 05 §6) | per view type |
| `C_S` | Task state | `tid task`, `id state` | the pool state of `task` equals `state` (§3.2) | — |
| `C_P` | Position 2D | `num x`, `num y`, `num yaw`, `num tol_xy`, `num tol_yaw` | horizontal distance ≤ `tol_xy` and (yaw within `tol_yaw`, or `tol_yaw < 0`) | `pose.enu`, `att.yaw` |
| `C_P3` | Position 3D | `num x`, `num y`, `num z`, `num tol_xy`, `num tol_z`, `num yaw`, `num tol_yaw` | horizontal distance ≤ `tol_xy` and `|Δz|` ≤ `tol_z` and (yaw within `tol_yaw`, or `tol_yaw < 0`) | `pose.enu`, `att.yaw` |
| `C_G` | Geo position | `num lat`, `num lon`, `num alt_amsl`, `num tol_xy`, `num tol_z` | as `C_P3` after conversion to ENU through the mission's geo reference, yaw ignored | `pose.enu` |
| `C_H` | Altitude | `num h`, `num tol` | `|alt.agl − h|` ≤ `tol` | `alt.agl` |
| `C_Q` | Path completed | reserved | | |

Rules:
* Angle differences are wrapped to (−π, π] before comparing.
* All tolerances MUST be ≥ 0, except `tol_yaw`, where a negative value means "ignore yaw".
* `C_T` and `C_W` use seconds. The 2021 core strings (`C_T 3000023`) used other units; the converter handles them.

Examples:

```
C_1: C_L AND C_1 C_2/
C_1: C_? ?_1/
?_1: airborne T/
C_2: C_m battery.remaining ge 0.3 0/
```

## 5. Requirements

Requirements say what a robot needs to take a task. They are checked by the dump rule (spec 06 §5).

| Code | Name | Slots | Satisfied when | Check |
|---|---|---|---|---|
| `R_S` | All of | `R+` | every child is satisfied | — |
| `R_F` | Fields | `num max_age`, `id+ fields` | every field exists and is at most `max_age` s old | static: the self model can provide it; dynamic: present and fresh now |
| `R_A` | Actions | `code+` | every action code has an actuator in the self model | static |
| `R_R` | Role | `id role` | the robot has the role (JB decision 9) | static |
| `R_E` | Resource | `id resource`, `num amount` | the resource manager can commit `amount` | dynamic |
| `R_K` | Affinity | `tid task` | this robot executed `task` (interdependent task) | dynamic |
| `R_C` | Runtime condition | `C` | the condition is TRUE on every tick from `STARTED` to the end | runtime (§8.5) |

**Implicit requirements.** Every field read by the task's conditions and functions, and every action code it uses, is added automatically as an `R_F` (with `max_age` = the field's default, spec 05 §3) and an `R_A`. A task author only writes the requirements that cannot be derived: roles, resources, affinity, runtime conditions, and fields read inside registry functions not covered by their registration.

## 6. Complex tasks and tree links

### 6.1 Execution on one robot

| Code | On one robot (version 0.1) |
|---|---|
| `T_S` | Runs the children in order. Each child starts when the previous one has `SUCCEEDED`. A failed child fails the parent |
| `T_L` | Runs the children one after another in listed order. The parent succeeds when `k` children have succeeded; the remaining children are cancelled. Running children at the same time on one robot is reserved for a later version (it needs actuator locks, plan §7.6) |
| `T_O` | Tries the children in order of effective priority, highest first, skipping those whose static requirements fail. The parent succeeds when one child succeeds |

The parent's start condition is checked before its first child starts, and its end condition after the last child ends.

### 6.2 Decomposition across robots

`ITaskDecomposer` turns a tree into **leaf tasks** that any allocator can take. The result MUST be the same on every robot, so it depends only on the tree.

1. **Ids.** Child `i` (1-based, in listed order) of task `p` gets the id `p.i`. Example: the second child of `op.17` is `op.17.2`, and its first child is `op.17.2.1`.
2. **Leaves.** Every `T_A`, `T_P` and `T_B` in the tree is a leaf. Complex tasks are never executed as a whole by one robot after decomposition. Their state is derived from their children.
3. **Precedence.** For a `T_S` parent, child `i+1` gets the extra start condition `C_S p.i DONE`, applied to the first leaf of child `i+1`. A leaf whose added conditions are false is `BLOCKED`.
4. **Parallel.** `T_L` children have no precedence. When `k` are `DONE`, the rest become `CANCELLED`.
5. **Choice.** `T_O` children are all `AVAILABLE`. The first one `DONE` cancels the others.
6. **Start and end of a complex parent.** The parent's start condition is added to the start conditions of its first leaves. Its end condition is checked by the robot that finishes its last leaf; if it is FALSE, the parent is `FAILED`.
7. **Effective priority** is inherited as in §2.
8. **Affinity** (`R_K`) is kept on each leaf as written.

Example: inspect an area in parallel strips, then land everyone. Two strips are shown.

```
T: T_S op.40 1 0 C_1 C_2 T_1 T_2/
C_1: C_N/
C_2: C_N/
T_1: T_L 0 1 0 0 C_1 C_2 T_1 T_2/
C_1: C_N/
C_2: C_N/
T_1: T_A 0 1 0 C_1 C_2 A_1/
C_1: C_P3 0 -50 20 1 0.5 0 -1/
C_2: C_N/
A_1: A_N/
T_2: T_A 0 1 0 C_1 C_2 A_1/
C_1: C_P3 0 50 20 1 0.5 0 -1/
C_2: C_N/
A_1: A_N/
T_2: T_A 0 1 0 C_1 C_2 A_1/
C_1: C_? ?_1/
?_1: landed T/
C_2: C_N/
A_1: A_N/
```

Decomposition gives three leaves: `op.40.1.1` and `op.40.1.2` (the strips, `AVAILABLE` to any robot) and `op.40.2` (land, `BLOCKED` until `op.40.1` is `DONE`). In a real mission each robot gets its own landing leaf; the generator writes one per robot.

*Note:* the ids inside a tree are written as `0` and assigned by the decomposer. A task file MUST give an id only to the root.

## 7. The behaviour library

A behaviour library maps **condition codes** to **behaviour tasks** that make that condition true (JB decision 4). It is a `.mrsb` file of `B` records.

| Code | Slots | Meaning |
|---|---|---|
| `B_E` | `id name`, `code fulfils`, `id qualifier`, `num priority`, `T behaviour` | Library entry. `fulfils` is a condition code (for example `C_P3`). `qualifier` narrows it: for `C_?` it is the predicate name that the behaviour makes TRUE (for example `airborne`); for every other code it is `any`. `behaviour` is normally a `T_B` |

Lookup: for an unmet condition with code `c`, the executor takes the entries with `fulfils = c` (and, for `C_?`, `qualifier` equal to the predicate name, with the predicate's wanted value `T`) whose static requirements the robot meets, and picks the highest `priority`. Ties go to the first entry in the file. A `C_?` condition that wants a predicate to be `F` has no behaviour in version 0.1; write it with the opposite predicate instead (`landed T`, not `airborne F`).

**Binding.** Before the chosen behaviour starts, the condition's values are written into the worldview as `target.*` fields (spec 05 §4). The behaviour's functions read those fields.

| Condition | Fields written |
|---|---|
| `C_P` | `target.x`, `target.y`, `target.yaw`, `target.tol_xy`, `target.tol_yaw`; `target.z` = current `pose.enu.z` |
| `C_P3` | `target.x`, `target.y`, `target.z`, `target.tol_xy`, `target.tol_z`, `target.yaw`, `target.tol_yaw` |
| `C_G` | as `C_P3`, after conversion to ENU; `target.yaw` = current yaw, `target.tol_yaw` = −1 |
| `C_H` | `target.z` = `h` + ground height, `target.tol_z` = `tol` |
| `C_?` | `target.predicate`, `target.value` |
| `C_W`, `C_T` | none; the `hover` entries hold position until the condition is TRUE |

Example entry: fly straight to a 3D point.

```
B: B_E flyto.direct C_P3 any 5 T_1/
T_1: T_B 0 1 0 C_1 C_2 C_3 T_1/
C_1: C_? ?_1/
?_1: airborne T/
C_2: C_N/
C_3: C_P3 0 0 0 0 0 0 -1/
T_1: T_P 0 1 0 C_1 C_2 A_1..2/
C_1: C_N/
C_2: C_N/
A_1: A_MAP any A_1 any A_2/
A_1: A_FN A_PXY F_1 F_2/
F_1: F_L 1 1 0 target.x/
F_2: F_L 1 1 0 target.y/
A_2: A_FN A_PZY F_1 F_2/
F_1: F_L 1 1 0 target.z/
F_2: F_X yaw_to_target 0/
A_2: A_N/
```

*Note:* the `C_3` in a library behaviour is a placeholder. When an entry is bound, its `until` condition is replaced by the unmet condition itself, so the behaviour runs until the thing it was called for is true. The layered fly-to has the same shape and uses the registry functions `layered_x`, `layered_y` and `layered_z` (spec 02 §8), which climb to the robot's layer first, cross at that altitude and descend at the target. The full library is [`examples/uav_behaviours.mrsb`](examples/uav_behaviours.mrsb).

## 8. The executor contract

The executor runs inside `RobotController::Tick` (plan §7.2). It owns an **active-task stack**. The top of the stack is the task being worked on.

### 8.1 One tick

On each tick, with worldview `w` and mission time `t`:

1. If the stack is empty, ask the MRS layer for a task. If there is none, do nothing.
2. Let `τ` be the top task. Evaluate its runtime conditions (§8.5). If any is FALSE or UNKNOWN, end `τ` as `FAILED` (§8.7).
3. If `τ` is `QUEUED` or `STARTED`: evaluate its start condition.
   * TRUE: set `τ` to `IN_PROGRESS` and continue with step 4 **in the same tick**.
   * FALSE: if `τ` is `QUEUED`, find a behaviour for the start condition (§7), bind it, push it, set `τ` to `STARTED`, and run step 2 for the pushed behaviour in the same tick. If `τ` is already `STARTED`, the behaviour on top of it is still running; do nothing more for `τ`.
   * UNKNOWN: do nothing this tick. After `start_unknown_timeout` (default 5 s), end `τ` as `FAILED` with reason `MISSING_FIELD`.
4. If `τ` is `IN_PROGRESS`: take its current action (§8.2), evaluate it if parametric, and dispatch it.
5. Write the journal if any task state changed (spec 06 §8).

Exactly **one** action (which may be an `A_MAP`) is dispatched per tick.

### 8.2 Actions and the iterator

* A leaf task has an **action iterator**, starting at the first action.
* The executor dispatches the action at the iterator. The actuator returns one of `DONE`, `RUNNING`, `REJECTED`, `FAILED`.
* `DONE`: the iterator moves to the next action, which is dispatched on the **next** tick.
* `RUNNING`: the same action is dispatched again on the next tick (re-evaluating functions). Long actions such as `A_TO`, `A_LD` and `A_HD` with a duration return `RUNNING` until they finish.
* `REJECTED` (no actuator accepts it): the task ends as `FAILED`, reason `NO_ACTUATOR`.
* `FAILED`: the task ends as `FAILED`, reason `ACTUATOR_FAILED`.
* Setpoint actions (`A_PXY`, `A_PZY`, `A_VXY`, `A_VZY`) and `A_L` return `DONE` as soon as they are accepted.
* `A_W d` is handled by the executor: it is `RUNNING` until at least `d` seconds have passed since it was first dispatched (**≥**, not >, fixing the 2021 inversion).
* If the iterator passes the last action, the task behaves as if it reached `A_N`.

### 8.3 The null action and the end of a task

When the iterator reaches `A_N`, the executor evaluates the end condition:

* TRUE: `SUCCEEDED`.
* FALSE: `FAILED`, reason `END_NOT_MET`.
* UNKNOWN: `FAILED`, reason `MISSING_FIELD`.

The task is popped. If the stack has another task below it, that task continues on the next tick.

The **context start** of `C_W` is the tick at which the task (or behaviour) that holds the condition entered `STARTED` or `IN_PROGRESS`, whichever came first.

### 8.4 Behaviour tasks (`T_B`)

* `until` is evaluated **every tick before** dispatching, while the behaviour is `IN_PROGRESS`. When it is TRUE, the behaviour stops at once, as if its base task reached `A_N`, and the end condition is evaluated.
* When the base task reaches `A_N` and `until` is still FALSE, the base task's iterator is reset and it runs again from its first action on the next tick.
* **Depth limit:** behaviours pushed to fulfil start conditions may nest at most `max_behaviour_depth` deep (default 2). Beyond that, the task fails with reason `NO_BEHAVIOUR`.
* A behaviour that ends `SUCCEEDED` returns control to the task below. That task's start condition is evaluated again on the next tick. If it is still FALSE, the lookup runs again; after `max_fulfil_attempts` (default 3) the task fails with reason `START_UNREACHABLE`.
* No behaviour for the condition code: the task fails with reason `NO_BEHAVIOUR`.

### 8.5 Runtime conditions

A task's `R_C` conditions, and those of every task below it on the stack, are evaluated every tick from `STARTED` to the end. A FALSE or UNKNOWN result ends the task as `FAILED`, reason `RUNTIME_CONDITION`, and pops every behaviour above it.

### 8.6 Preemption and resume

* An interrupt (plan §7.3) or the allocator can **preempt** the top task by pushing another task on top. The preempted task becomes `IDLE`. Its action iterator, the start of its `C_W` contexts and its behaviour stack are kept.
* When the task above it ends, an `IDLE` task becomes `QUEUED` again and its **start condition is evaluated again**. A fly-to that was interrupted halfway therefore continues from where the robot is.
* The allocator can also **replace** a task: the current task becomes `IDLE` and is returned to the pool as `AVAILABLE` (spec 06 §5).

### 8.7 Failure reasons

| Reason | Cause |
|---|---|
| `NO_BEHAVIOUR` | No library entry for the start condition, or the depth limit was reached |
| `START_UNREACHABLE` | The behaviour ended but the start condition stayed FALSE after the allowed attempts |
| `END_NOT_MET` | The end condition was FALSE at the null action |
| `MISSING_FIELD` | A condition or function needed a field that was missing or stale |
| `RUNTIME_CONDITION` | A runtime condition became FALSE or UNKNOWN |
| `NO_ACTUATOR` | No actuator accepts an action |
| `ACTUATOR_FAILED` | The actuator reported failure |
| `IMPOSSIBLE` | The task executed `A_I` |
| `PREEMPTED_OUT` | The task was replaced and returned to the pool |
| `ABORTED` | The operator aborted it |

`FAILED` tasks go to the dump rule (spec 06 §5), which decides whether another robot may try them.

### 8.8 Parameters of the executor

| Parameter | Default | Meaning |
|---|---|---|
| `max_behaviour_depth` | 2 | §8.4 |
| `max_fulfil_attempts` | 3 | §8.4 |
| `start_unknown_timeout` | 5 s | §8.1 |
