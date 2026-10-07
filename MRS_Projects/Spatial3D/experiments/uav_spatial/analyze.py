#!/usr/bin/env python3
"""WP6 analysis (plan §10.5, spec 10 §7): reads the CSV that mrs_experiment writes and prints
Markdown tables.

  python3 analyze.py results.csv [--out results.md] [--boot 2000]

Speed-up is paired: for each task set and seed, makespan(S1) / makespan(condition). Means come
with 95% bootstrap intervals (percentile method, resampling the paired runs, fixed RNG seed so
the tables are reproducible). Only the Python standard library is used.
"""
import argparse
import csv
import math
import random
import statistics
import sys
from collections import defaultdict

GROUPS = ["G-RTA", "G-RTA-X", "G-C"]


def load(path):
    rows = []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            for k in ("tasks", "seed", "n", "completed", "done", "failed", "duplicates", "busy_robots", "messages", "bytes",
                      "plan_calls", "separation_breaches", "fence_exits", "crashed"):
                r[k] = int(r[k])
            for k in ("makespan_s", "latency_mean_s", "latency_max_s", "distance_m", "energy_wh", "decision_mean_us",
                      "decision_worst_us", "plan_worst_ms", "min_separation_m", "sim_time_s", "wall_s"):
                r[k] = float(r[k])
            r["source"] = r["set"].split("/")[0]
            rows.append(r)
    return rows


def boot_ci(values, boot, rng):
    if len(values) < 2:
        return (float("nan"), float("nan"))
    n = len(values)
    means = sorted(sum(values[rng.randrange(n)] for _ in range(n)) / n for _ in range(boot))
    return means[int(0.025 * boot)], means[min(boot - 1, int(0.975 * boot))]


def fmt(x, d=2):
    return "–" if x is None or (isinstance(x, float) and math.isnan(x)) else f"{x:.{d}f}"


def table(header, rows):
    out = ["| " + " | ".join(header) + " |", "|" + "|".join("---" for _ in header) + "|"]
    out += ["| " + " | ".join(str(c) for c in r) + " |" for r in rows]
    return "\n".join(out)


def cond_key(r):
    return (r["condition"], r["n"])


def cond_name(k):
    return k[0] if k[0] in ("S1", "S1*") else f"{k[0]} N={k[1]}"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--out")
    ap.add_argument("--boot", type=int, default=2000)
    a = ap.parse_args()
    rows = load(a.csv)
    rng = random.Random(20261007)
    by_run = {}
    for r in rows:
        by_run[(r["set"], r["seed"], r["condition"], r["n"])] = r
    conds = sorted({cond_key(r) for r in rows}, key=lambda k: (["S1", "S1*"] + GROUPS).index(k[0]) * 100 + k[1])
    ns = sorted({k[1] for k in conds if k[0] in GROUPS})
    sources = sorted({r["source"] for r in rows})
    out = []
    w = out.append

    seeds = sorted({r["seed"] for r in rows})
    sets = sorted({r["set"] for r in rows})
    w(f"Runs: {len(rows)} over {len(sets)} task sets, seeds {seeds[0]}–{seeds[-1]}. "
      f"Stalled runs (time limit reached): {sum(1 for r in rows if not r['completed'])}.\n")

    # 1. Makespan per condition and source.
    w("## Makespan (s)\n")
    w("Mean over task sets and seeds, with the 95% bootstrap interval of the mean.\n")
    hdr = ["Condition"] + [f"{s}" for s in sources]
    body = []
    for k in conds:
        line = [cond_name(k)]
        for s in sources:
            v = [r["makespan_s"] for r in rows if cond_key(r) == k and r["source"] == s]
            if not v:
                line.append("–")
                continue
            lo, hi = boot_ci(v, a.boot, rng)
            line.append(f"{statistics.mean(v):.1f} [{lo:.1f}, {hi:.1f}] (n={len(v)})")
        body.append(line)
    w(table(hdr, body) + "\n")

    # 2. Paired speed-up against S1.
    def speedups(k, pred):
        v = []
        for r in rows:
            if cond_key(r) != k or not pred(r):
                continue
            s1 = by_run.get((r["set"], r["seed"], "S1", 1))
            if s1 and r["makespan_s"] > 0:
                v.append(s1["makespan_s"] / r["makespan_s"])
        return v

    def speed_table(title, groups):
        w(f"## {title}\n")
        w("Paired speed-up makespan(S1) / makespan(condition): mean [95% CI].\n")
        hdr = [""] + [f"{g} N={n}" for g in GROUPS for n in ns]
        body = []
        for label, pred in groups:
            line = [label]
            for g in GROUPS:
                for n in ns:
                    v = speedups((g, n), pred)
                    if not v:
                        line.append("–")
                        continue
                    lo, hi = boot_ci(v, a.boot, rng)
                    line.append(f"{statistics.mean(v):.2f} [{lo:.2f}, {hi:.2f}]")
            body.append(line)
        w(table(hdr, body) + "\n")

    speed_table("Speed-up by source", [(s, (lambda s: lambda r: r["source"] == s)(s)) for s in sources])
    gen = [r for r in rows if r["source"] == "gen"]
    if gen:
        fams = sorted({r["family"] for r in gen})
        disps = sorted({r["dispatch"] for r in gen})
        speed_table("Speed-up by family (generated sets)",
                    [(f, (lambda f: lambda r: r["source"] == "gen" and r["family"] == f)(f)) for f in fams])
        speed_table("Speed-up by dispatch (generated sets)",
                    [(d, (lambda d: lambda r: r["source"] == "gen" and r["dispatch"] == d)(d)) for d in disps])

    # 3. The oracle against the online single UAV.
    w("## S1* against S1\n")
    w("Paired ratio makespan(S1*) / makespan(S1); below 1 means knowing the timeline helps.\n")
    body = []
    for s in sources:
        v = []
        for r in rows:
            if r["condition"] == "S1*" and r["source"] == s:
                s1 = by_run.get((r["set"], r["seed"], "S1", 1))
                if s1 and s1["makespan_s"] > 0:
                    v.append(r["makespan_s"] / s1["makespan_s"])
        if v:
            lo, hi = boot_ci(v, a.boot, rng)
            body.append([s, f"{statistics.mean(v):.3f} [{lo:.3f}, {hi:.3f}]", len(v)])
    w(table(["Source", "S1*/S1", "Pairs"], body) + "\n")

    # 4. Secondary metrics.
    w("## Secondary metrics\n")
    w("Means over all runs of a condition. Messages and bytes are on the shared channel; decision "
      "time is the robot-side allocator call; plan time is the central planner (G-C) or the robot's "
      "own replans (S1, S1*).\n")
    hdr = ["Condition", "Latency mean (s)", "Latency max (s)", "Distance (m)", "Energy (Wh)", "Duplicates",
           "Busy robots", "Messages", "kB", "Decision mean (µs)", "Decision worst (µs)", "Plan worst (ms)",
           "Sep. breaches", "Min sep. (m)", "Fence exits", "Stalled", "Crashed"]
    body = []
    for k in conds:
        v = [r for r in rows if cond_key(r) == k]
        m = lambda f: statistics.mean(r[f] for r in v)
        seps = [r["min_separation_m"] for r in v if r["min_separation_m"] >= 0]
        body.append([cond_name(k), fmt(m("latency_mean_s"), 1), fmt(m("latency_max_s"), 1), fmt(m("distance_m"), 0),
                     fmt(m("energy_wh"), 1), fmt(m("duplicates"), 2), fmt(m("busy_robots"), 1), fmt(m("messages"), 0),
                     fmt(m("bytes") / 1000, 1), fmt(m("decision_mean_us"), 1), fmt(max(r["decision_worst_us"] for r in v), 0),
                     fmt(max(r["plan_worst_ms"] for r in v), 1), sum(r["separation_breaches"] for r in v),
                     fmt(min(seps), 2) if seps else "–", sum(r["fence_exits"] for r in v),
                     sum(1 for r in v if not r["completed"]), sum(r["crashed"] for r in v)])
    w(table(hdr, body) + "\n")

    # 5. The best group condition per N.
    w("## Best condition per team size\n")
    body = []
    for n in ns:
        best = None
        for g in GROUPS:
            v = speedups((g, n), lambda r: True)
            if v and (best is None or statistics.mean(v) > best[1]):
                best = (g, statistics.mean(v))
        if best:
            body.append([n, best[0], f"{best[1]:.2f}", f"{best[1] / n:.2f}"])
    w(table(["N", "Best", "Speed-up", "Per robot"], body) + "\n")

    text = "\n".join(out)
    if a.out:
        with open(a.out, "w", newline="\n") as f:
            f.write(text)
    else:
        sys.stdout.write(text)


if __name__ == "__main__":
    main()
