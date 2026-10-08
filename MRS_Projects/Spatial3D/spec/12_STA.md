# 12 Splittable trees and MRS-STA

Status: **WP8**, spec version `0.1`. Plan reference: §8.5 (allocators, MRS-STA), §8.7 (tree tasks split between robots), §10 (evaluation), §11 (WP8). Builds on specs [03](03_Tasks_and_Execution.md) §6 (complex tasks), [09](09_MRS_Layer.md) (MRS layer, MRS-RTA), [10](10_Evaluation.md) (evaluation) and [11](11_Trees_and_Generators.md) (trees across robots, generators).

## 1. Scope

WP8 is the end goal of the MRS layer (plan §11): a tree task is split into base tasks and shared between robots under STA. It adds:

* **splittability**: a tree is split only where it can be split. The decomposer stops at **units**, the parts one robot takes from the pool (§2);
* **MRS-STA**: MRS-RTA-X over units with an active-task stack and a preference for units near the robot's last one in the tree (§3);
* **evaluation v2**: tree-task sets, and G-STA beside S1, G-RTA and G-RTA-X (§4).

JB (2026-10-08, with the WP8 go-ahead): "The splitting of the task should be done only with tasks that can be split." §2.1 is how the library reads that.

Code: `TaskTree.h` (units, `Splittable`), `Allocator.h` (`StaAllocator`), the MRS layer and issuer in `algorithms/`; `sim/src/TreeSets.cpp`; `tools/mrs_experiment.cpp`.

## 2. Units

### 2.1 What can be split

A node of a tree can be split when its children can go to different robots without breaking a rule of spec 03 §6.2:

| Node | Can be split |
|---|---|
| leaf (`T_A`, `T_P`, `T_B`) | never: it is a base task |
| `T_L`, `T_O` | always: its children are independent |
| `T_S` | unless **every** child after the first is bound to the child before it by affinity, and no child can be split |

A `T_S` child is *bound to the child before it* when each of its first leaves (spec 11 §2.1) has an `R_K` whose partner lies under the previous child. A bound sequence is one robot's work by construction: the second step may only be done by the robot that did the first, so splitting it would only add a handover between pool entries. Examples from the WP7 generators:

* a waypoint chain (spec 11 §3.2): every leg after the first has `R_K` to the leg before it, so the chain is one unit;
* coverage (§3.3): a `T_L` of cell chains, so each cell is a unit;
* release at a point (§3.1): "land there", then "release" with `R_K` to the landing, so the pair is one unit;
* a search (§3.5): a `T_L` of spiral chains, so each spiral is a unit;
* `ThenLand` (§3.6): a `T_S` of the tree and the landings `T_L`, which can be split.

A `T_S` whose later children are free (no `R_K`, or an `R_K` to a leaf that is not under the previous child) can be split: its children become separate units, with precedence (`after`) between them as in WP7.

`Splittable(tree, id)` gives the rule. `Decompose` marks the units top-down from the root: a node that cannot be split is a unit, and the decomposer does not go below it. `TaskTree::units` lists them in depth-first order; `unit_of` maps every leaf to its unit. A tree whose root cannot be split is one unit, the root itself.

### 2.2 Units in the pool

The MRS layer adds every unit, not every leaf, to its pool (spec 11 §2.3 is amended by this). A leaf unit is as in WP7. A **complex unit** is the subtree record with the decomposer's ids written into it (child *i* of `p` is `p.i`, at every level), so the robot that takes it runs it as one complex task (spec 03 §6.1, §8) and the executor reports every leaf by its tree id. A complex unit's

* priority is the product of the priorities from the root down to the unit (spec 03 §6.2 rule 7);
* `after` is its first leaf's `after` (only nodes outside the unit can appear there, because a unit holds only leaves and bound `T_S` nodes);
* `gates` are the start conditions of complex ancestors above the unit whose first leaves include the unit's first leaf;
* affinity is its first leaf's `R_K` when that points outside the unit; an `R_K` inside the unit is kept by running it on one robot.

`UpdateTrees` gates units instead of leaves, derives the state of the nodes above the units only (a unit and the nodes inside it are the business of the robot running it, which also checks their end conditions), and cancels the units under a node that has ended.

**Leaves inside a complex unit.** The robot controller passes the executor's events for the children of a complex task to the MRS layer (they stay out of the journal). When a leaf inside the unit this robot runs succeeds, the robot sends `M_DONE <leaf id>` once, and every robot that hears it records who did the leaf (`leaf_done_by`). So

* an affinity partner may be a leaf inside a complex unit: the bound entry is gated until `leaf_done_by` names this robot;
* the issuer follows every leaf (`Leaves()`) as in WP7 and every complex unit (`Units()`), and derives the root from the units;
* `StateOf(leaf)` is `DONE` once a robot reported it, otherwise the unit's state when the unit has failed, otherwise `AVAILABLE`.

**When a complex unit counts as started.** For the allocators (`CurrentTask::started`), a complex unit is started once its first leaf runs (`IN_PROGRESS`), not when the robot sets off to it. Until then, the robot may still give way to one that is closer.

### 2.3 Not in WP8

* A robot that takes over a complex unit (after a failure or a swap) runs it from its first leaf; it does not skip leaves another robot already reported.
* Splitting a long bound chain in the middle (for example, half a cell each) is not done: by §2.1 the chain is one robot's work.

## 3. MRS-STA

### 3.1 The allocator

`StaAllocator` is MRS-RTA-X (spec 09 §4.3) with a different ranking. It always claims (exclusive mode). Its score for a pool entry is

```
score = base_priority × size_est × (1 + stack_bonus)^[tree on stack] × (1 + relative_bonus × kinship)
```

* **Active-task stack.** When the robot selects a unit of a tree, it pushes the tree's root (or moves it to the top). A root stays on the stack while any unit of its tree is not finished, and is popped at the next `Select` after that. The stack is the robot's set of engaged trees: a robot finishes the tree it is in before it starts another (`stack_bonus` = 0.5).
* **Kinship** with the unit this robot finished last: the share of the unit's path below the root that the two have in common (`op.1.2.3` against `op.1.2.1`: 1 of 2 parts, 0.5). Units in the same branch as the last one rank higher (`relative_bonus` = 0.5); siblings directly under the root have kinship 0.
* Tasks outside trees rank as in MRS-RTA-X.

Subtask states are shared through the messages the MRS layer already sends: claims (`peer_active`) and unit and leaf `M_DONE` (`peer_done`). STA needs no message of its own.

### 3.2 Started tasks keep their claim (amends spec 09 §4.3)

In exclusive mode, the stronger claim keeps a task, until the task's actions run. A robot whose task has started keeps it even when another robot now scores higher, and while it works, the score it claims with does not drop below the score it claimed with when it started. Before WP8, the started task was dropped: a robot flying a cell moved away from the cell's first waypoint, its distance term fell, and a robot waiting near the start outclaimed it, so the cell was started again from its first leg. On a mixed tree set this took one G-STA run from 129 s to 281 s. The change applies to MRS-RTA-X too. For atomic tasks it makes no measurable difference, because a robot whose actions run is at the target and already has the top score: re-flying 45 WP6 runs (cluster, random and multicluster sets, G-RTA-X at N = 3 and 8, G-RTA at N = 5) gave 40 identical runs; the other 5 (all G-RTA-X at N = 8) moved by −2.5% to +2.5%, and the condition means by at most 0.5%.

### 3.3 Where it runs

`Sim::Condition::G_STA` ("G-STA") in `RunOne` and `mrs_experiment`, and `mrs_uav <id> sta` in Webots.

## 4. Evaluation v2

### 4.1 Tree-task sets

`GenerateTreeSet(TreeGenConfig)` writes a timeline of `trees` tree tasks from the WP7 generators, released `interval` s apart (default 20 s from 1 s), with each tree at a random altitude of 10 to 20 m inside ±60 m:

| Family | Tree | Units per tree |
|---|---|---|
| `coverage` | a rotated rectangle of 30 to 50 m a side, footprint 8 m, `parts` cells | `parts` cell chains |
| `perimeter` | a rotated rectangle of 40 to 80 m a side, legs up to 10 m, `parts` arcs | `parts` arc chains |
| `search` | `parts` points of interest, spiral radius 10 m, footprint 8 m | `parts` spiral chains |
| `mixed` | coverage, perimeter, search in turn | `parts` each |

`mrs_experiment run --trees FAMILY:TREES:PARTS[:INTERVAL]` adds them to the grid as `tree/<family>-<trees>x<parts>[-i<interval>]/s<seed>`. The CSV gains `leaves` (leaves of split trees) and `leaf_duplicates` (leaf `M_DONE` beyond the first). `busy_robots` also counts robots that only did leaves.

### 4.2 Grid

Five sets (coverage, perimeter, search and mixed with 3 trees of 6 parts, 20 s apart; mixed with 6 trees of 4 parts, 5 s apart, so trees overlap), seeds 1 to 10, conditions S1 (one robot, the tree run whole, spec 11 §2.6), G-RTA, G-RTA-X and G-STA at N = 2, 3 and 5. Results are in `experiments/uav_spatial/results/wp8/` (`results.csv`, its `.meta.txt`, and `tables.md` from `analyze.py`).

### 4.3 Results

500 runs, no stalled runs, crashes or fence exits, no leaf done twice under G-RTA-X or G-STA (`results/wp8/README.md`, `tables.md`). Paired speed-up makespan(S1) / makespan(group) over all tree sets, mean:

| | N = 2 | N = 3 | N = 5 |
|---|---|---|---|
| G-RTA | 1.81 | 2.38 | 3.07 |
| G-RTA-X | 1.84 | 2.43 | 3.17 |
| G-STA | 1.81 | 2.40 | 3.11 |

G-STA is level with or 1–5% slower than G-RTA-X (paired ratio 0.99 to 1.05 per set and N). Units are already one robot's work, the size estimate already sends a robot to the nearest unit, and the generated trees are one level deep, so the stack and kinship have little to add. Splitting itself is what pays: search sets reach 3.7–3.8 at N = 5, perimeters 2.3–2.5.

## 5. Tests (WP8)

`tests/sta/test_sta.cpp` (spec 07 §5.10):

* a waypoint chain is one unit; a coverage `T_L` splits into its cells; a cell unit's record carries the tree ids at every level;
* a `T_S` of free leaves splits; land-then-release (bound) does not; an `R_K` to a leaf that is not under the previous child splits;
* a complex unit's priority product, `after`, gates and affinity; a leaf inside a unit asks the caller for its state;
* kinship values;
* a chain flown by one of two robots: every leaf reported once, all by the robot that did the root;
* G-STA with two robots on a 4-cell coverage: the tree is on a robot's stack while it has units, every stack is empty afterwards, both robots take cells;
* tree-task sets are deterministic, parse, release 20 s apart and have 6 units per tree;
* **acceptance** (plan §11 WP8): mixed set 3 × 6, seed 2, G-STA with three UAVs completes with every robot busy, no leaf done twice, no fence exit or crash, and under 0.6 × the makespan of one UAV.

The WP7 acceptance test now also checks that each coverage cell is one unit.
