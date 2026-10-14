# WP8 results: tree tasks split between UAVs, MRS-STA against MRS-RTA

Plan §10, spec 12 §4. QuadSim. 500 runs over 50 tree-task sets (5 set kinds × 10 seeds), no stalled runs, no crashes, no geofence exits.

| File | What it is |
|---|---|
| `results.csv` | One row per run (spec 10 §5, plus `leaves` and `leaf_duplicates`, spec 12 §4.1) |
| `results.csv.meta.txt` | Mission, noise, battery, coefficients |
| `tables.md` | Every table from `analyze.py results.csv` |

Sets: coverage, perimeter, search and mixed, each 3 trees of 6 units released 20 s apart; and mixed with 6 trees of 4 units released 5 s apart, so several trees are open at once. Conditions: S1 (one UAV runs each tree whole) and G-RTA, G-RTA-X and G-STA at N = 2, 3 and 5.

## Answer

**Splitting trees into units works: every group condition finishes tree sets well ahead of one UAV.** No leaf was done twice under G-RTA-X or G-STA; open G-RTA did a leaf twice in 3 of 150 runs. Paired speed-up makespan(S1) / makespan(group), all tree sets, mean [95% CI]:

| | N = 2 | N = 3 | N = 5 |
|---|---|---|---|
| G-RTA | 1.81 [1.76, 1.85] | 2.38 [2.31, 2.45] | 3.07 [2.94, 3.20] |
| G-RTA-X | 1.84 [1.80, 1.88] | 2.43 [2.35, 2.49] | 3.17 [3.03, 3.31] |
| G-STA | 1.81 [1.77, 1.85] | 2.40 [2.33, 2.46] | 3.11 [2.97, 3.26] |

Search sets gain most (3.7–3.8 at N = 5: six short spirals per tree), perimeters least (2.3–2.5: the arcs are long and few robots fit on one boundary).

**MRS-STA does not beat MRS-RTA-X on these sets.** The paired ratio makespan(G-STA) / makespan(G-RTA-X) is 0.99 to 1.05 per set and team size; most intervals include 1, and where they do not, G-STA is 1–5% slower. Why:

* Units are already large. A cell, arc or spiral is one robot's work (spec 12 §2.1), so there is little left for "prefer the next unit of my tree" to gain: the distance term of the size estimate already sends a robot to the nearest unit, which is usually in the tree it is working on.
* The stack bonus sometimes keeps a robot in its tree when a unit of a newer tree is closer, and kinship has little to work with: the generators' trees are one level deep (all units are siblings, kinship 0) except under `ThenLand`.
* With overlapping trees (mixed 6 × 4, 5 s apart), where the stack should matter most, G-STA is level with G-RTA-X (0.99–1.01).

Exclusive claims matter more than the stack: open G-RTA has about 2–4 times the separation breaches of G-RTA-X and G-STA at every N (shared targets, as in WP6), and travels 5–13% further than G-RTA-X.

## Not measured

* Deeper trees (units under several levels of `T_S`/`T_L`), where kinship would separate near and far relatives.
* Tuning `stack_bonus` and `relative_bonus` (both 0.5 here).
* Battery swaps in the middle of a unit (spec 12 §2.3).
