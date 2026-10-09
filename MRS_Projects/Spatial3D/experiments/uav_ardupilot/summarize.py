# summarize.py RESULTS_DIR [QUADSIM_CSV]: one row per SITL run (a directory holding results.csv),
# with the QuadSim run of the same condition beside it when a QuadSim results CSV is given.
import csv, os, sys
from collections import Counter

CONDITION = {"rta": "G-RTA", "rta-x": "G-RTA-X", "cbba": "G-CBBA", "ldta2": "G-LDTA2", "sta": "G-STA"}

root = sys.argv[1]
quadsim = {}
if len(sys.argv) > 2:
    for r in csv.DictReader(open(sys.argv[2])):
        quadsim[(r["condition"], int(r["n"]))] = r

print("| run | N | done | makespan s | mean latency s | max latency s | tasks per robot | QuadSim makespan s | SITL / QuadSim |")
print("|---|---|---|---|---|---|---|---|---|")
for name in sorted(os.listdir(root)):
    path = os.path.join(root, name, "results.csv")
    if not os.path.exists(path) or "_n" not in name:  # run directories are named <set>_n<N>_<alloc>
        continue
    rows = list(csv.DictReader(open(path)))
    done = [r for r in rows if r["done"]]
    lat = [float(r["done"]) - float(r["dispatch"]) for r in done]
    # As TaskIssuer::Makespan: last completion minus first dispatch.
    makespan = max(float(r["done"]) for r in done) - min(float(r["dispatch"]) for r in rows) if done else float("nan")
    per = Counter(r["done_by"] for r in done)
    n = int(name.split("_n")[1].split("_")[0])
    alloc = name.split("_", 2)[2]
    cond = "S1" if n == 1 else CONDITION.get(alloc, alloc)
    q = quadsim.get((cond, n))
    qm = float(q["makespan_s"]) if q else None
    print(f"| {name} | {n} | {len(done)}/{len(rows)} | {makespan:.1f} | {sum(lat)/len(lat):.1f} | {max(lat):.1f} | "
          f"{' '.join(f'{k}:{v}' for k, v in sorted(per.items()))} | {qm if qm is None else f'{qm:.1f}'} | "
          f"{'' if qm is None else f'{makespan/qm:.2f}'} |")
