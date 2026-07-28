#!/usr/bin/env python3
"""Paired per-seed comparison of two bench result CSVs.

Same seeds run under two configurations are highly correlated, so the paired
difference has a far smaller standard error than either mean on its own.
Usage: cmp.py <base.csv> <variant.csv>
"""
import sys, statistics as st

import os
SCRATCH = os.environ.get("BENCH_OUT", os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")) + "/"


def load(p):
    if not p.endswith(".csv"):
        p = SCRATCH + p + ".csv"
    out = {}
    for line in open(p):
        f = line.strip().split(",")
        if len(f) > 4:
            out[int(f[0])] = (int(f[4]), int(f[3]))  # completed, discovered
    return out


a, b = load(sys.argv[1]), load(sys.argv[2])
seeds = sorted(set(a) & set(b))
for i, name in ((0, "completed"), (1, "discovered")):
    da = [a[s][i] for s in seeds]
    db = [b[s][i] for s in seeds]
    d = [y - x for x, y in zip(da, db)]
    n = len(d)
    m = st.mean(d)
    se = st.stdev(d) / (n ** 0.5) if n > 1 else 0.0
    flag = "**" if abs(m) > 2 * se else ("*" if abs(m) > se else "")
    print(f"{name:11s} {st.mean(da):6.2f} -> {st.mean(db):6.2f}   diff {m:+.3f} +/- {se:.3f} {flag}")
print(f"(n={len(seeds)} paired seeds; ** = outside 2 standard errors)")
