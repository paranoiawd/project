# HANDOFF — state of the work, for the next session

**Mission from here on: on the ORIGINAL configuration (16 tasks), discover as many tasks as
possible and complete as many as possible. The ideal run is 16 discovered / 16 completed.
Completed is the primary metric; discovered is the enabler.**

Read this file before touching code. It records what is already known, what already works, and —
importantly — what has already been tried and measured to *fail*, so none of it is redone.

---

## 1. Ground rules (do not violate)

* Only `schedular.h` and `schedular.cpp` count as the deliverable. The grader compiles them
  against the **originally provided** `main.cpp` / `simulator.{h,cpp}`. Anything else you add
  must not be required for the scheduler to build and run.
* Target language level is **C++14** (MSVC `cl`). Verify with
  `g++ -std=c++14 -w -I bench/shim -fsyntax-only schedular.cpp`.
* `main.cpp` on `main` is the pristine original (`NUM_MAX_TASKS = 16`). Keep it that way; use
  `bench/bench` for any other configuration instead of editing it.

## 2. Simulator mechanics (already derived from the code — do not re-derive)

| Fact | Value |
|---|---|
| Step u→v cost | `ceil((floor(cost(u)/2) + cost(v)) / 10)` ticks, 10 energy/tick |
| Consequence | **energy and time are the same currency**; 1 tick = 10 energy |
| Work cost | `10 * max(1, ceil(task_cost / 10))` energy |
| HOLD | free (no energy, no time) |
| Drone task cost | `INFINITE` — a drone that starts a task burns all its energy and dies. Never let it. |
| View | drone 5×5, caterpillar 3×3, wheel cross-5. Exhausted robots stop observing. |
| Travel dominance | a cross-map trip ≈ 3000–6000 energy vs ~100 for the work itself. Routing *is* the problem. |

**Original configuration constants** (`MAP_SIZE=20`, `NUM_ROBOT=6`, `WALL_DENSITY=20`):
`TIME_MAX = 2000`, energy/robot `= 12000`, `NUM_MAX_TASKS = 16`, initial tasks `= 8`,
remaining 8 spawn at **t = 500, 625, 750, …, 1375** (`TIME_MAX/4` then every `TIME_MAX/16`).
Robots: ids 0,3 = drone; 1,4 = caterpillar; 2,5 = wheel.

Budget arithmetic worth remembering: 4 workers × 12000 = **48k energy total** for all travel +
work; 2 drones × 12000 = **24k**, which buys roughly **1.25 full sweeps of the map** — that is the
hard reason full observation is impossible.

## 3. Where performance stands today (measured, 500 fresh seeds each unless noted)

Original config (cap 16), current `main` HEAD, regime-adaptive scheduler:

| Metric | Value |
|---|---|
| Discovered | **14.03 / 16** |
| Completed | **8.72 / 16** |
| Discovered → completed conversion | **62 %** |

Same code forced into the completion-tuned profile (`SCHED_FORCE_REGIME=1`, ≈ the ac88b21 tuning):

| Metric | Value |
|---|---|
| Discovered | 10.90 / 16 |
| Completed | **9.73 / 16** |
| Conversion | **89 %** |

Provided placeholder scheduler (random walk), same config: ~7 discovered, **~1 completed**.

**Oracle probe** (perfect information — every cell observed every tick, current scheduler,
200 seeds): completed **12.22 / 16** avg, max 16, `P(=16) = 0.5 %`, `P(≥14) = 15 %`.
Caveat: the oracle run still spends drone/worker energy on now-pointless observation, so the true
routing ceiling is somewhat above 12.2.

### The central open problem

Discovery gains did **not** convert. +3.1 discovered bought **−1.0 completed**. The observation
work (drone rim ring, worker debt patrols, worker energy reserved for observation) is paid for out
of the same 48k pool that completions need. Meanwhile the oracle says that even with free perfect
information the current routing leaves ~3.8 tasks unfinished.

So there are two independent gaps to close, and the second is currently the larger one:
1. **Discovery gap** ≈ 2 tasks (14.0 → 16).
2. **Conversion gap** ≈ 3.8 tasks (routing/energy, visible even under perfect information).

## 4. Architecture of `schedular.cpp` (navigation map)

`Scheduler::State` holds all planning state; everything is replanned each tick in
`on_info_updated`, which runs in this order:

1. **regime detection** — `st.regime`: 0 undecided → 1 dense (completion profile) / 2 sparse
   (discovery profile), decided at t=130 by discovered count, with structurally-safe corrections.
   `SCHED_FORCE_REGIME=1|2` pins it (debug only; harmless in the deliverable).
2. **task snapshot + magnet grid** — free tasks mark `st.magnet` so routed paths get an edge
   discount over them (`TASK_MAGNET`) and workers collect tasks *en route*.
3. **per-robot Dijkstra** — two maps per worker: `dist` (magnet-routed, used for *paths*) and
   `dist_c` (clean, used for *costing*). Mixing them up silently corrupts economics.
4. **matching** — `server_count`/`sole_server`, then greedy over marginal-cost score
   `need * (1 + need/energy)`, waiting-clock promotion (`STARVE_AGE`), cheapest-insertion tour
   preference (`TOUR_FIRST_BONUS`), time-feasibility cut, endgame spend-down (`ENDGAME_TICKS_DEF`),
   fragmentation guard measured from the *candidate task's* cell.
5. **observation valuation** — `serve_dist` (cheapest worker reach per cell) weights `build_staleness`
   so cells that are cheap to serve are worth more to observe.
6. **drones** — dense: paced adaptive stale-window crawl. sparse: late rim ring, start time
   back-computed from the seed's real drone step cost, arc pre-trimmed to affordable length,
   leftover arc inheritable by the other drone; parked as a fixed camera when out of budget.
7. **workers idle** — patrol on staleness, switching to *debt* mode (cells unseen since the last
   spawn, excluding what a drone route will still cover) after `NEED_T_PER_CELL * n`.

`on_task_reached` is the safety gate: never drones, never a task someone is already working,
never without enough energy to finish, opportunistic takeover only when strictly cheaper.
`idle_action` just converts `next_step` into a direction and refuses to walk into known walls.

## 5. What worked (keep these)

* Exact step-cost Dijkstra (not Manhattan, not uniform cost).
* Split routed/clean distance maps.
* Task magnet on worker paths — free pickups en route.
* Marginal-cost scoring — pushes medium hauls onto energy-rich robots.
* Waiting clock + sole-server protection — stops both over-eager far trips and starvation.
* Cheapest-insertion tour preference — kills zigzag.
* Serve-cost-weighted observation — the single biggest completion gain.
* Drone commitment (never re-evaluate a goal you are still walking to) — removes oscillation.
* Ring start time derived from the seed's own drone cost; arc trimmed to what energy can pay.

## 6. What was tried and measured to FAIL (do not redo)

| Attempt | Measured effect |
|---|---|
| Hard commitment matching (keep pairs until infeasible) | −0.7 completed; churn is not the bottleneck |
| Round-2 "guarantee every affordable task an owner" | −1.2 completed; drains fleet on far trips |
| Worker quadrant zone anchoring + drift-to-center | −0.3, and idle workers wander for nothing |
| Interior tile-lattice endgame sweep (fixed waypoint grid) | −0.3 discovered, −0.8 completed |
| Fixed interior lane sweep for the debt phase | −0.2 discovered vs argmax patrol |
| Cooperative ring-cycle split by nearest ring point | −0.7 discovered (long approach legs) |
| Consuming a ring waypoint once merely *in view* | −1.5 discovered (skips the far side of the arc) |
| Patrol window claiming between workers | ±0, added complexity |
| Removing the suppression stack entirely ("naked economics") | −1.0 completed under partial info, ±0 under oracle |
| Horizontal drone lanes instead of vertical | worse; bands are tall and narrow |

## 7. Suggested attack order for the next session

1. **Decouple the profiles.** Discovery layer and task economics are currently coupled through
   `st.regime`. The obvious untested combination is the *sparse observation layer* running on the
   *dense task economics* (no `DEBT_RESERVE` withholding, dense serve-weights). Cheap to try, and
   it directly targets the 62 % → 89 % conversion collapse.
2. **Attack the conversion gap with the oracle.** Run `./bench <seed> 16 oracle` and study why 3.8
   tasks die even with perfect information — that is pure routing/energy allocation, no observation
   noise. Fixing it lifts every configuration at once.
3. **Only then chase the last ~2 discoveries**, and only with observation that pays for itself
   (workers observing *along* task routes rather than on dedicated patrols).
4. Re-measure on ≥300 seeds after each change; single-seed impressions have misled repeatedly.
   Always report both discovered and completed — optimizing one alone has already backfired once.

## 8. Bench harness

```bash
bench/build.sh                       # g++ -O2, conio.h shim for Linux
cd bench
./bench <seed> [cap] [-v|oracle]     # one run; CSV: seed,cap,created,discovered,completed,
                                     #   exhausted,end_time,workerE,droneE,droneCellCost
./bench 45 16 -v                     # + per-task spawn/discovery/affordability, miss classifier
                                     #   (never_seen vs timing), robot economics, known map dump
./bench 45 16 oracle                 # perfect information (isolates routing from observation)
./run.sh 300 16                      # 300 seeds in parallel + aggregate
python3 aggregate.py results.csv 15  # P(metric >= threshold) with Wilson lower bound
```

Always validate on seeds you did **not** tune on (tuning used 1–400; holdouts used 10001+, 20001+,
30001+, 40001+, 60001+, 70001+).

## 9. History of the branch

| Commit | What it is | cap52 completed P(≥18) | cap16 discovered P(≥15) |
|---|---|---|---|
| `ac88b21` | completion-focused scheduler | **93.4 %** | 2.4 % |
| `c843d0d` | + discovery layer, regime switch | 88.8 % | 36.4 % |
| `a55c064` | discovery-first tuning + ring triage | 82–84 % | **43–46 %** |

The `NUM_MAX_TASKS = 52` experiments existed only to answer an earlier goal ("18+ completions at
90 %", impossible at cap 16 since only 16 tasks ever exist). That goal is closed. `main.cpp` is back
to the original 16 and the mission is now the original configuration only. `REPORT.md` keeps the
full curve and methodology.
