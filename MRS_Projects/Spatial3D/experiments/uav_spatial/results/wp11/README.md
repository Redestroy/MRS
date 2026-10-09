# WP11 results: repulsion in QuadSim (spec 15 §5)

QuadSim, `mrs_experiment` at the WP11 commit. The 30-task generated sets (5 families × even and static
dispatch × seeds 1–5 = 50 sets), N = 8, open (G-RTA) and exclusive (G-RTA-X) MRS-RTA, each flown with the
repulsion off (lead 0) and used by the safety supervisor with a lead of 1 s and 2 s (spec 15 §4.3).
The lead 1 and 2 runs were flown again with every robot carrying the ranging sensor (0.1 m noise), so the
repulsion reads measured relative positions instead of peer states. 500 runs, all completed, no fence exits,
no crashes. Layers 3 m apart, GPS noise 0.05 m. `tables.py` writes `tables.md`.

A breach is two airborne robots closer than 1 m (spec 10 §5), counted once per approach.

| condition | lead | breaches / run | runs with a breach | makespan vs lead 0 |
|---|---|---|---|---|
| G-RTA | off | 12.9 | 47 / 50 | 1 |
| G-RTA | 1 s | 8.1 | 45 / 50 | 1.000 ± 0.008 |
| G-RTA | 2 s | 3.0 | 35 / 50 | 0.998 ± 0.009 |
| G-RTA-X | off | 2.1 | 27 / 50 | 1 |
| G-RTA-X | 1 s | 1.3 | 23 / 50 | 1.000 ± 0.007 |
| G-RTA-X | 2 s | 1.1 | 18 / 50 | 1.007 ± 0.007 |

* With a 2 s lead the repulsion removes 77% of the breaches under open MRS-RTA and about half under
  exclusive MRS-RTA, and the mean closest approach rises (open: 0.24 m to 0.88 m). Makespan does not change
  (within 1%).
* It does not remove them all. Under open MRS-RTA the remaining breaches are most likely robots sent to the same target (inferred from the WP6 traces, not traced again here):
  the task's position condition pulls both to one point, and the push only slows the approach. Exclusive
  claims, or real avoidance (WP12), are the fix for that.
* Measured relative positions (ranging, 0.1 m noise) give the same result as peer states here, because
  QuadSim's GPS is good (0.05 m) and peer states arrive every 0.5 s. The sensor matters where GNSS is poor
  or the radio is slow, which this grid does not test.

The hook is a measurement aid, off by default; WP12 replaces it with an avoidance strategy.
