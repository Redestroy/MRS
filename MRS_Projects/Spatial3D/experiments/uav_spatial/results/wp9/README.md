# WP9 results: CBBA, LDTA² and a limited radio (evaluation v3)

Plan §10, spec 13 §6. QuadSim. 1460 runs: 730 with no bitrate limit and the same 730 at 9.6 kbit/s. No run stalled, crashed or left the fence.

| File | What it is |
|---|---|
| `results.csv` | One row per run (spec 10 §5, spec 12 §4.1, plus `bitrate_bps`, `dropped`, `delay_mean_s`, `delay_max_s`) |
| `results.csv.meta.txt` | Mission, noise, battery, coefficients. There is one block per `mrs_experiment` call: the unlimited half and the 9.6 kbit/s half were flown separately, with the same build apart from how duplicates are counted |
| `tables_unlimited.md` | `analyze.py results.csv --bitrate 0` |
| `tables_9600.md` | `analyze.py results.csv --bitrate 9600` |

**Sets.** The flat sets are generated with 30 tasks each: random, cluster and multicluster, released evenly or all at the start ("static"). The tree sets are mixed 3 × 6 (20 s apart) and mixed 6 × 4 (5 s apart). Every set is flown with seeds 1 to 5, which gives 40 sets.

**Conditions.** S1, then G-RTA, G-RTA-X, G-STA, G-CBBA, G-LDTA2 and G-C at N = 3, 5 and 8. G-C runs only on the flat sets, because it plans no tree leaves (spec 11).

## Answer

**With no limit on the radio, every decentralized allocator does the same on flat sets.** Paired speed-up makespan(S1) / makespan(group), with the 95% CI for N = 8:

| | N = 3 | N = 5 | N = 8 |
|---|---|---|---|
| G-RTA | 2.28 | 2.92 | 3.32 [3.05, 3.59] |
| G-RTA-X | 2.30 | 2.95 | 3.33 [3.04, 3.60] |
| G-STA | 2.30 | 2.95 | 3.33 [3.06, 3.59] |
| G-CBBA | 2.26 | 2.94 | 3.33 [3.07, 3.60] |
| G-LDTA2 | 2.30 | 2.95 | 3.33 [3.06, 3.60] |
| G-C (central) | 2.29 | 2.99 | 3.46 [3.14, 3.80] |

On a flat set every task has the same LDTA² type, so G-LDTA2 is G-RTA-X by construction (spec 13 §5.3). G-C keeps its lead on static sets at N = 8 (4.35 against about 4.03).

**On tree sets with no limit, RTA-X and CBBA lead and LDTA² trails at small N:**

| | N = 3 | N = 5 | N = 8 |
|---|---|---|---|
| G-RTA | 2.44 | 3.20 | 3.80 |
| G-RTA-X | 2.49 | 3.32 | 4.14 |
| G-STA | 2.49 | 3.27 | 4.02 |
| G-CBBA | 2.34 | 3.27 | 4.09 |
| G-LDTA2 | 2.20 | 3.01 | 4.05 |

The likely cause (inferred from the rule, not traced run by run): LDTA² holds robots to their tree's share. When few robots are spread over several open trees, a robot waits on its own tree instead of taking a nearer unit in another one. By N = 8 there are robots enough for every tree, and the gap closes.

**At 9.6 kbit/s CBBA loses the most.** Speed-up at N = 8:

| | Flat sets | Tree sets |
|---|---|---|
| G-RTA | 2.81 | 2.56 |
| G-RTA-X | 3.06 | 2.57 |
| G-STA | 3.06 | 2.61 |
| G-CBBA | 2.52 | 2.19 |
| G-LDTA2 | 3.06 | 2.33 |
| G-C | 3.01 | – |

CBBA's speed-up at N = 8 is no better than at N = 5: 2.53 against 2.52 on flat sets, and 2.21 against 2.19 on tree sets. Its bid tables grow with the number of robots and tasks, so at 9.6 kbit/s its bids arrive late. At N = 8 its mean queueing delay is 3.3 s, against 2.4 s for RTA-X. On a limited link G-C falls back level with RTA-X.

**Safety (separation breaches per run, N = 8):**

| | Unlimited | 9.6 kbit/s |
|---|---|---|
| G-RTA | 479 | 1005 |
| G-RTA-X | 85 | 386 |
| G-STA | 85 | 398 |
| G-CBBA | 54 | 1048 |
| G-LDTA2 | 86 | 417 |
| G-C | 0 | 11 |

With a free channel, CBBA has the fewest breaches of the decentralized allocators. Its consensus gives each task one winner, so robots share targets less often. On the limited link this advantage reverses, because stale bids let two robots fly to the same task (2.75 duplicate completions per run at N = 8, against 2.08 for RTA-X and 0.03 for G-C).

**Message cost.** A bitrate of 9.6 kbit/s drops between 5 and 520 messages per run on average (G-C N = 3 fewest, G-LDTA2 N = 8 most). The repeat rules of spec 13 §3.3 recover the losses: no run stalled. With no limit, CBBA sends 30% fewer messages than RTA-X at N = 8 (3358 against 4844) but about the same bytes (623 kB against 613 kB), because each of its messages carries a table.

## Notes

* **How duplicates are counted.** A duplicate is now a task reported done by more than one robot. On a lossy link a robot re-sends its `M_DONE` (spec 13 §3.3), and those repeats are not counted. Before this was fixed, S1 showed 1.8 duplicates per run at 9.6 kbit/s, which were only repeats. The unlimited half does not resend, so its counts did not change. The 9.6 kbit/s half was re-flown after the fix, and its makespans did not change.
* **Leaf duplicates at 9.6 kbit/s are real.** Up to 5.4 per run at N = 8 (G-RTA), meaning a lost claim or `M_DONE` let a second robot fly a leaf.
* **Decision time.** CBBA takes 12–38 µs per call against RTA-X's ~80 µs, most likely because a call that finds its bundle still valid does little work.
