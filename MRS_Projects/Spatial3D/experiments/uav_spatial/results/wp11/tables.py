# tables.py [results.csv]: the WP11 repulsion tables (spec 15 §5) as Markdown on stdout.
# Groups by condition, repulsion lead and ranging noise; paired differences use the same set and seed.
import csv
import math
import sys
from collections import defaultdict

path = sys.argv[1] if len(sys.argv) > 1 else "results.csv"
rows = list(csv.DictReader(open(path)))


def key(r):
    return (r["condition"], float(r["repulsion_lead_s"]), float(r["ranging_noise_m"]))


def mean_ci(xs):
    n = len(xs)
    m = sum(xs) / n
    if n < 2:
        return m, 0.0
    sd = math.sqrt(sum((x - m) ** 2 for x in xs) / (n - 1))
    return m, 1.96 * sd / math.sqrt(n)


groups = defaultdict(list)
for r in rows:
    groups[key(r)].append(r)

print("| condition | lead (s) | ranging | runs | completed | breaches / run | runs with a breach | min separation (m) | makespan (s) | makespan vs lead 0 |")
print("|---|---|---|---|---|---|---|---|---|---|")
base = {(r["condition"], r["set"], r["seed"]): float(r["makespan_s"]) for r in rows
        if float(r["repulsion_lead_s"]) == 0.0 and float(r["ranging_noise_m"]) < 0}
for k in sorted(groups):
    g = groups[k]
    cond, lead, noise = k
    b, bci = mean_ci([float(r["separation_breaches"]) for r in g])
    with_breach = sum(1 for r in g if float(r["separation_breaches"]) > 0)
    seps = [float(r["min_separation_m"]) for r in g if float(r["min_separation_m"]) >= 0]
    ms, msci = mean_ci([float(r["makespan_s"]) for r in g])
    ratios = [float(r["makespan_s"]) / base[(cond, r["set"], r["seed"])] for r in g if (cond, r["set"], r["seed"]) in base]
    ratio = mean_ci(ratios) if ratios else (float("nan"), 0.0)
    print(f"| {cond} | {lead:g} | {'off' if noise < 0 else f'{noise:g} m noise'} | {len(g)} | {sum(int(r['completed']) for r in g)} | "
          f"{b:.2f} ± {bci:.2f} | {with_breach} | {min(seps) if seps else float('nan'):.2f} (mean {sum(seps) / len(seps):.2f}) | "
          f"{ms:.1f} ± {msci:.1f} | {ratio[0]:.3f} ± {ratio[1]:.3f} |")
