# 13 CBBA, LDTA² and the communication budget

Status: **WP9**, spec version `0.1`. Plan reference: §8.5 (allocators), §8.6 (communication), §10 (evaluation), §11 (WP9). Builds on specs [06](06_Messages.md) (messages), [09](09_MRS_Layer.md) (MRS layer), [10](10_Evaluation.md) (evaluation) and [12](12_STA.md) (units, MRS-STA).

## 1. Scope

WP9 adds the two comparison allocators of JB's scope and makes the radio's limits part of the evaluation:

* **CBBA**, the consensus-based bundle algorithm (§2);
* the **communication budget**: a bitrate-limited channel in the test simulator, and robots that pace their own messages to it (§3);
* **LDTA²**, the local dynamic task allocation algorithm (§5);
* **evaluation v3**: every allocator on the same sets, with no limit and at 9.6 kbit/s (§6).

Code: `Cbba.h`, `Ldta2.h` and the MRS layer in `algorithms/`; `Air` in `sim/Team.h`; `Sim::Condition::G_CBBA`, `G_LDTA2`; `mrs_experiment --bitrate`.

## 2. CBBA

Choi, Brunet and How (2009). Each robot runs two phases every `Select`.

### 2.1 Bundle construction

The robot keeps a **bundle** (tasks in the order it added them, at most `bundle` = 3) and a **path** (the same tasks in the order it will fly them). The score of a path is

```
S(path) = Σ_j priority_j × exp(−T_j / horizon)
```

where `T_j` is the arrival time at task *j* along the path from where the robot is now: the layered travel time of spec 10 §2.1 (`TravelModel`), plus the settle time and the task's hold time for every task before it, and `horizon` = 100 s. The score has diminishing marginal gain, which CBBA needs to converge.

While the bundle is not full, the robot computes for every eligible task not in its bundle the best marginal gain `c = S(path ⊕_k j) − S(path)` over every insertion place *k*, keeps those that **beat** the task's winning bid (§2.2), adds the one with the largest gain to the bundle and inserts it at its best place in the path. Its bid for that task is `c`.

A task whose actions have started is **locked**: the robot raises its bid to 10⁶ and keeps the task first in the path, so no robot takes it over (the same rule as spec 12 §3.2 for claims).

### 2.2 Consensus

Every robot keeps a table: for each task, the winning bid `y`, the winner `z` (empty for none) and the time `s` the winner made the bid. A bid **beats** an entry when it is higher, or equal and from a lower robot number; anything beats an empty entry or one whose winner is no longer alive (spec 06 §5 peer timeout).

The robot sends its table in `M_BID` (spec 06 §4): the entries that changed, at most every `bid_period` = 0.5 s, and the whole table every `full_period` = 5 s. On an `M_BID` from robot *k*, for each entry `(j, y, z, s)`:

| Entry from *k* | This robot's entry | Result |
|---|---|---|
| winner is this robot | any | ignored: only a robot speaks for its own bids |
| no winner | winner *k*, older | reset to no winner (*k* gave it up) |
| winner *z* | winner *z*, older | updated (newer word from the same winner, which may be lower) |
| winner *z* | other winner, or none | updated when `(y, z)` beats it |

After merging, the robot finds the first task in its bundle that it no longer wins, and releases that task and every task added after it (their bids assumed the lost one). It gives up its own bids on those later tasks, which it sends as entries without a winner. The bundle is then built again.

This is the timestamped variant of the CBBA decision table that version 0.1 implements. The full table of Choi et al. also tracks, per robot pair, when information was last relayed. That table matters on multi-hop networks. The test simulator's channel reaches everyone, so version 0.1 leaves it out.

### 2.3 Selection

`Select` returns the first task of the path that is still eligible, and IDLE when there is none. A task that has finished leaves the table, the bundle and the path.

### 2.4 Not in version 0.1

* Tree units are scored by their first target, as in spec 12. CBBA plans units, not the leaves inside them.
* Bids are built from the robot's position at bundle time and are not revised while the robot flies. A rebuild happens when the robot loses a task, finishes one, or learns of a new one.

## 3. The communication budget

### 3.1 The channel

`Sim::Air` has an optional `bitrate_bps`. With a limit, it is one shared channel, first come first served. Each tick it carries messages from the head of the queue for as long as it has credit, and it earns `bitrate / 8` bytes of credit per second. Unused credit is saved for 0.1 s, or for as long as the message at the head needs, so a large message (a tree's `M_TASK`) does not block the channel. A message that has waited more than `max_delay` = 5 s before it goes on the air is lost (`dropped`); the message at the head is on the air and is never dropped, however long it takes (a tree's `M_TASK` can be several kilobytes). In the first full grid, dropping it too stalled every tree run at 9.6 kbit/s. The CSV reports `bitrate_bps`, `dropped`, `delay_mean_s` and `delay_max_s`.

### 3.2 The robots

`MrsConfig::bitrate_bps` tells the MRS layer the channel's bitrate (0: no limit). Each agent's **share** is `bitrate / (live peers + this robot + the issuer)`, and allocators read it from `AllocatorContext::budget_bps`.

* **Heartbeats.** `M_STATE` (about 200 bytes) takes at most `state_share` = 0.25 of the share: its period is `max(state_period, 200 × 8 / (0.25 × share))`.
* **CBBA bids.** `M_BID` takes at most `budget_share` = 0.5 of the share: its period is `max(bid_period, size × 8 / (0.5 × share))`, with size estimated as 80 + 40 bytes per entry.

### 3.3 Lost messages

A limited channel loses messages, and before WP9 nothing was sent twice. Two rules repair what is lost:

* the issuer repeats open tasks every 15 s (`IssuerConfig::task_period`) when the channel has a limit, and with them the `M_PLAN` of its last replan, unchanged (G-C; without it every static-dispatch G-C run at 9.6 kbit/s stalled);
* a robot that hears `M_TASK` for a task it has done sends its `M_DONE` again (for a tree root, the units it did), so a lost `M_DONE` does not leave the task open for ever.

Without them, the first trials stalled in every run at 1.2 and 2.4 kbit/s and in the static-dispatch runs at 9.6 kbit/s: a lost `M_TASK` or `M_DONE` was never sent again.

## 4. Conditions

| Condition | Allocator |
|---|---|
| `G-CBBA` | `CbbaAllocator` with the condition's travel model |
| `G-LDTA2` | `Ldta2Allocator` (§5) |

`mrs_experiment run --bitrate B1,B2,...` flies every run of the grid at each bitrate. The run key is set, seed, condition, N and bitrate.

## 5. LDTA²

### 5.1 Source

Local Dynamic Task Allocation, from de Mendonça, Nedjah and de Macedo Mourelle, "Efficient distributed algorithm of dynamic task assignment for swarm robotics", Neurocomputing 172 (2016) 345–355. The paper is not in the project. This section follows JB's bachelor thesis §2.1.1, pp. 21–24 and figs. 3–5 (flowcharts redrawn from the paper). A search of the inputs (notes in `E:\Claude\MRS\LDTA2_Notes.md`) found no difference between "LDTA" and "LDTA²".

### 5.2 The algorithm

Tasks `t_1 … t_τ` are task **types** that many robots share. `P` gives the wanted share of robots per type (Σ p = 1), and `C[t] = p_t × ρ` the wanted number of the ρ robots. Each robot *i* keeps its own type `t_i`, every robot's type `A_i` as it last heard it, and the counts `C_A[t]` from `A_i`. Each round:

1. **Update.** The robot sends its `(id, t_i)` and reads every other robot's, and counts `C_A`.
2. If `C_A[t_i] ≤ C[t_i]`, it performs a task of `t_i`.
3. Otherwise **AdjustTask**: `δ[t] = max(0, C[t] − C_A[t])` for every type, `t' = argmax δ` (ties: the first type), and `NR` = the robots with a lower number than *i* whose type is over-staffed (they move first). If `δ[t'] > NR`, the robot changes to `t'`.

There is no probability. The ordering by robot number keeps all robots from moving at once.

### 5.3 In MRS

* **Types.** Every split tree (spec 12) is one type, named by its root; every other task is type `*`.
* **Shares.** `P` is proportional to each type's open tasks (pool entries not finished and not dumped by this robot), so the team spreads over the open trees in proportion to their remaining units. `C` is rounded by largest remainder (ties: the first type) so that it adds up to ρ, and never exceeds a type's open tasks.
* **Update** uses the heartbeats the robots already send: a peer's type is the type of the task in its `M_STATE`. A robot without a task (idle, or on a type no longer open) counts as on no type, and is over-staffed.
* **Within a type**, the robot picks its task as MRS-RTA-X does (`Allowed` narrows `RtaAllocator::Select` to the type). A robot whose turn has not come keeps to its type; an idle one takes the best task of any type, so no robot idles while there is work.

On a set whose tasks are all of one type (the flat WP6 sets), LDTA² is MRS-RTA-X.

## 6. Evaluation v3

### 6.1 Grid

The flat sets are the WP6 generators with 30 tasks: random, cluster and multicluster, released evenly or all at the start. The tree sets are the WP8 mixed 3 × 6 and mixed 6 × 4 (5 s apart). Every set is flown with seeds 1 to 5. Conditions: S1, then G-RTA, G-RTA-X, G-STA, G-CBBA, G-LDTA2 and G-C (flat sets only) at N = 3, 5 and 8. Each run is flown with no bitrate limit and at 9.6 kbit/s, which makes 1460 runs. Results are in `experiments/uav_spatial/results/wp9/` (`results.csv`, its `.meta.txt`, `tables_unlimited.md`, `tables_9600.md` and `README.md`).

### 6.2 Results

No run stalled, crashed or left the fence. Paired speed-up makespan(S1) / makespan(group), mean:

| | Flat, unlimited | Flat, 9.6 kbit/s | Trees, unlimited | Trees, 9.6 kbit/s |
|---|---|---|---|---|
| G-RTA N = 3 / 5 / 8 | 2.28 / 2.92 / 3.32 | 2.22 / 2.63 / 2.81 | 2.44 / 3.20 / 3.80 | 1.94 / 2.29 / 2.56 |
| G-RTA-X | 2.30 / 2.95 / 3.33 | 2.28 / 2.81 / 3.06 | 2.49 / 3.32 / 4.14 | 2.06 / 2.46 / 2.57 |
| G-STA | 2.30 / 2.95 / 3.33 | 2.28 / 2.81 / 3.06 | 2.49 / 3.27 / 4.02 | 2.12 / 2.42 / 2.61 |
| G-CBBA | 2.26 / 2.94 / 3.33 | 2.21 / 2.53 / 2.52 | 2.34 / 3.27 / 4.09 | 1.95 / 2.21 / 2.19 |
| G-LDTA2 | 2.30 / 2.95 / 3.33 | 2.28 / 2.81 / 3.06 | 2.20 / 3.01 / 4.05 | 1.80 / 2.33 / 2.33 |
| G-C | 2.29 / 2.99 / 3.46 | 2.17 / 2.71 / 3.01 | – | – |

* **No limit.** On flat sets the decentralized allocators are level, and G-LDTA2 is G-RTA-X by §5.3. G-C leads at N = 8 (4.35 on static sets). On trees, RTA-X and CBBA lead. LDTA² trails at N = 3 and 5 because it holds robots to their tree's share, and it catches up at N = 8. CBBA has the fewest separation breaches of the decentralized allocators at N = 8 (54 per run, against 85 for RTA-X and 479 for RTA).
* **9.6 kbit/s.** CBBA stops gaining past N = 5. Its bid tables queue: at N = 8 its mean delay is 3.3 s against 2.4 s for RTA-X, it has 2.75 duplicate completions per run against 2.08, and 1048 breaches against 386. RTA-X and STA lose the least. G-C is level with RTA-X on flat sets.

A duplicate is a task reported done by more than one robot. A re-sent `M_DONE` (§3.3) is not counted.

### 6.3 Choices accepted

JB accepted these on 2026-10-09: the LDTA² type mapping (§5.3), the timestamped CBBA without the relay table (§2.2), the channel model (§3.1), the issuer repeat on limited links and the `M_DONE` re-send (§3.3).

## 7. Tests (WP9)

`tests/cbba/test_cbba.cpp` (spec 07 §5.11):

* CBBA's bid rule (higher, then lower robot number; no winner never beats);
* the limited channel delivers at its bitrate, lets a large head message save up, and drops a message that waited too long;
* G-CBBA with 3 UAVs on a static cluster set: every task done once, every robot busy, under 0.6 × one UAV;
* at 9.6 kbit/s G-CBBA and G-RTA-X still finish, with messages queued and fewer messages than without a limit;
* LDTA²'s wanted counts (largest remainder, capped by open tasks);
* G-LDTA2 with 3 UAVs on six overlapping trees: every tree done, every robot busy, no leaf done twice.
