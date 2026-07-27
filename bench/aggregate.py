#!/usr/bin/env python3
"""Aggregate bench CSV: seed,cap,created,discovered,completed,exhausted,end_time"""
import sys
import statistics as st

path = sys.argv[1]
target = int(sys.argv[2]) if len(sys.argv) > 2 else 18
rows = []
for line in open(path):
    line = line.strip()
    if not line:
        continue
    parts = line.split(",")
    rows.append({
        "seed": int(parts[0]), "cap": int(parts[1]), "created": int(parts[2]),
        "discovered": int(parts[3]), "completed": int(parts[4]),
        "exhausted": int(parts[5]), "end_time": int(parts[6]),
        "workerE": int(parts[7]) if len(parts) > 7 else -1,
        "droneE": int(parts[8]) if len(parts) > 8 else -1,
    })

n = len(rows)
comp = [r["completed"] for r in rows]
disc = [r["discovered"] for r in rows]
created = [r["created"] for r in rows]
ok = sum(1 for c in comp if c >= target)
rate = ok / n if n else 0.0
# Wilson 95% lower bound
import math
z = 1.96
if n:
    ph = rate
    lo = (ph + z*z/(2*n) - z*math.sqrt((ph*(1-ph) + z*z/(4*n))/n)) / (1 + z*z/n)
else:
    lo = 0.0

print(f"runs={n}  P(completed>={target}) = {rate:.3f}  (95% Wilson lower bound {lo:.3f})")
print(f"completed: mean={st.mean(comp):.2f} min={min(comp)} p5={sorted(comp)[max(0,int(0.05*n)-1)]} "
      f"median={st.median(comp)} max={max(comp)}")
print(f"discovered: mean={st.mean(disc):.2f} min={min(disc)}   created: mean={st.mean(created):.2f}")
we = [r["workerE"] for r in rows if r["workerE"] >= 0]
de = [r["droneE"] for r in rows if r["droneE"] >= 0]
if we:
    print(f"leftover energy: worker mean={st.mean(we):.0f} drone mean={st.mean(de):.0f} "
          f"(of 4x12000 / 2x12000)")
missed = [r["created"] - r["discovered"] for r in rows]
undone = [r["discovered"] - r["completed"] for r in rows]
print(f"not discovered: mean={st.mean(missed):.2f}  discovered-but-not-done: mean={st.mean(undone):.2f}")
hist = {}
for c in comp:
    hist[c] = hist.get(c, 0) + 1
print("hist:", " ".join(f"{k}:{v}" for k, v in sorted(hist.items())))
fails = sorted((r for r in rows if r["completed"] < target), key=lambda r: r["completed"])
if fails:
    print(f"fail seeds ({len(fails)}):", " ".join(
        f"{r['seed']}(c{r['completed']}/d{r['discovered']}/x{r['exhausted']}/t{r['end_time']})"
        for r in fails[:25]))
