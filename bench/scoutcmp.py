#!/usr/bin/env python3
"""Aggregate bench/scoutbound output with PAIRED per-seed differences.

The tool writes one line per seed:
  SCOUT,seed,discovered,completed,V_actual,V_workers,V_greedy,V_opt,V_opt_fix,
        drone_steps,step_ticks,n_open,pace0,pace1

Everything interesting here is a paired difference on the same seed, so report
the mean of the per-seed difference with its own standard error -- comparing two
column means throws away the pairing and inflates the error bar.

Usage: python3 scoutcmp.py out/scoutbound_980k.csv
"""
import sys
import statistics as st


def mean_se(xs):
    n = len(xs)
    if n < 2:
        return (xs[0] if xs else 0.0), 0.0
    return st.mean(xs), st.stdev(xs) / (n ** 0.5)


def main(path):
    rows = []
    for line in open(path):
        line = line.strip()
        if not line.startswith("SCOUT,"):
            continue
        f = line.split(",")
        rows.append(dict(
            seed=int(f[1]), disc=int(f[2]), comp=int(f[3]),
            v_actual=float(f[4]), v_workers=float(f[5]), v_greedy=float(f[6]),
            v_opt=float(f[7]), v_opt_fix=float(f[8]),
            steps=int(f[9]), sc=int(f[10]), n_open=int(f[11]),
            pace0=int(f[12]), pace1=int(f[13]),
            best_k=int(f[14]), best_band=int(f[15]),
            unseen_g=int(f[16]), unseen_o=int(f[17]),
            late_g=int(f[18]), late_o=int(f[19]),
            sum_g=int(f[20]), sum_o=int(f[21]),
        ))
    n = len(rows)
    if n == 0:
        print("no rows")
        return
    print(f"n = {n} seeds\n")

    disc = st.mean(r["disc"] for r in rows)
    comp = st.mean(r["comp"] for r in rows)
    va = st.mean(r["v_actual"] for r in rows)
    print("MODEL VALIDITY (does the expected-discovery model reproduce reality?)")
    print(f"  real discovered           {disc:6.2f}")
    print(f"  model V on the real run   {va:6.2f}      (want these close)")
    print(f"  real completed            {comp:6.2f}\n")

    print("LEVELS (expected tasks discovered)")
    for k, lab in [("v_workers", "workers only, drones deleted"),
                   ("v_actual", "the real run"),
                   ("v_greedy", "greedy tours, FULL map knowledge"),
                   ("v_opt_fix", "optimised tours, pace fixed at 1900"),
                   ("v_opt", "optimised tours, pace free")]:
        m, se = mean_se([r[k] for r in rows])
        print(f"  {lab:38s} {m:6.3f} +/- {se:.3f}")
    print()

    print("PAIRED DIFFERENCES (the numbers that decide candidate A)")
    for a, b, lab in [
        ("v_greedy", "v_actual", "map knowledge alone (NOT candidate A)"),
        ("v_opt_fix", "v_greedy", "TOUR OPTIMISATION, pace held equal"),
        ("v_opt", "v_opt_fix", "re-pacing on top (known frontier trade)"),
        ("v_opt", "v_greedy", "both together"),
        ("v_actual", "v_workers", "what the drones currently contribute"),
    ]:
        d = [r[a] - r[b] for r in rows]
        m, se = mean_se(d)
        wins = sum(1 for x in d if x > 1e-9)
        star = "  <-- " if a == "v_opt_fix" and b == "v_greedy" else "      "
        print(f"{star}{lab:42s} {m:+6.3f} +/- {se:.3f}   "
              f"({m/se if se > 0 else 0:5.1f} SE, better on {wins}/{n})")
    print()

    p = [r["pace0"] for r in rows] + [r["pace1"] for r in rows]
    print(f"pace chosen by the optimiser: mean {st.mean(p):.0f}, "
          f"median {st.median(p):.0f}  (shipped DRONE_PACE_T = 1900)")
    print(f"drone steps per run: {st.mean(r['steps'] for r in rows):.0f}, "
          f"step cost {st.mean(r['sc'] for r in rows):.1f} ticks, "
          f"open cells {st.mean(r['n_open'] for r in rows):.0f}")
    kk = {}
    for r in rows:
        kk[(r["best_k"], r["best_band"])] = kk.get((r["best_k"], r["best_band"]), 0) + 1
    print("greedy baseline variant that won (locality_k, banded): " +
          ", ".join(f"{k}:{v}" for k, v in sorted(kk.items(), key=lambda t: -t[1])))
    print()

    print("SHAPE OF THE GAP -- is the optimiser covering MORE cells, or LATER ones?")
    ug, _ = mean_se([r["unseen_g"] for r in rows])
    uo, _ = mean_se([r["unseen_o"] for r in rows])
    lg, _ = mean_se([r["late_g"] for r in rows])
    lo, _ = mean_se([r["late_o"] for r in rows])
    mg, _ = mean_se([r["sum_g"] / max(1, r["n_open"] - r["unseen_g"]) for r in rows])
    mo, _ = mean_se([r["sum_o"] / max(1, r["n_open"] - r["unseen_o"]) for r in rows])
    dun, dun_se = mean_se([r["unseen_o"] - r["unseen_g"] for r in rows])
    dla, dla_se = mean_se([r["late_o"] - r["late_g"] for r in rows])
    print(f"  open cells never observed at all   greedy {ug:6.1f}   opt {uo:6.1f}"
          f"   diff {dun:+.2f} +/- {dun_se:.2f}")
    print(f"  cells observed at/after t=1375     greedy {lg:6.1f}   opt {lo:6.1f}"
          f"   diff {dla:+.2f} +/- {dla_se:.2f}")
    print(f"  mean last-observation tick         greedy {mg:6.0f}   opt {mo:6.0f}")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "out/scoutbound_980k.csv")
