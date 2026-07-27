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
* Every claim needs ≥ 300 seeds behind it, and **both** metrics reported. Use
  `bench/cmp.py` — the paired per-seed difference has ~5× the resolution of comparing two means,
  which matters because most single changes here are worth 0.02–0.15 completions.

## 2. Simulator mechanics (already derived from the code — do not re-derive)

| Fact | Value |
|---|---|
| Step u→v cost | `ceil((floor(cost(u)/2) + cost(v)) / 10)` ticks, 10 energy/tick |
| Consequence | **energy and time are the same currency**; 1 tick = 10 energy |
| Work cost | `10 * max(1, ceil(task_cost / 10))` energy |
| HOLD | free (no energy, no time) |
| Drone task cost | `INFINITE` — a drone that starts a task burns all its energy and dies. Never let it. |
| View | drone 5×5, caterpillar 3×3, wheel cross-5. Exhausted robots stop observing. |
| Terrain | `cat = 2t+100`, `wheel = 4t+50` from the same per-cell `t`, so **wheel = 2·cat − 150**: cheap ground is cheap for both, and the wheel gains more from routing onto it. Drone cost is one uniform value per map (120–198). |
| Travel dominance | a cross-map trip ≈ 3000–6000 energy vs ~100 for the work itself. Routing *is* the problem. |

**Original configuration constants** (`MAP_SIZE=20`, `NUM_ROBOT=6`, `WALL_DENSITY=20`):
`TIME_MAX = 2000`, energy/robot `= 12000`, `NUM_MAX_TASKS = 16`, initial tasks `= 8`,
remaining 8 spawn at **t = 500, 625, 750, …, 1375** (`TIME_MAX/4` then every `TIME_MAX/16`).
Robots: ids 0,3 = drone; 1,4 = caterpillar; 2,5 = wheel.

**The budget is the whole problem.** A robot carries 6 energy per tick of the run, so it can act
for at most **60 % of the run** — 40 % of all robot-time *must* be spent holding. Four workers ×
12000 = 48k buys roughly **16 average task trips**; two drones × 24k buys roughly **1.5 sweeps of
the map** (≈ 50 steps each at ~240 energy/step, 5 new cells per step, ~320 non-wall cells).
Observation and completion are paid for out of the same pool.

## 3. Where performance stands today (measured)

Current `main` HEAD, original config, **holdout seeds 920001–920500** (never used for tuning):

| Metric | Value | Pre-session HEAD |
|---|---|---|
| Completed | **10.58** | 8.83 |
| Discovered | 12.49 | 14.11 |
| Conversion | 85 % | 63 % |
| P(completed ≥ 11) | **54.4 %** | 21.4 % |
| P(completed ≥ 12) | **29.4 %** | 7.2 % |
| Leftover worker energy | 4.0k / 48k | 6.1k |

Provided placeholder scheduler (random walk): ~7 discovered, **~1 completed**.

## 3a. THE CEILING — read this before accepting any completion target

`bench/plan.cpp` solves each instance **offline and to convergence**: it knows the
whole map, every task's position and release time, routes four workers optimally with
insertion + relocate/swap/2-opt over hundreds of restarts, and **spends nothing at all on
observation**. It is not a scheduler; it exists to separate "our router is short" from
"the fleet cannot".

**500 seeds, no clairvoyant pre-positioning** (a worker may not set off toward a task
before that task exists — the one concession to realism):

| | |
|---|---|
| Mean completed | **14.12 / 16** |
| P(= 16) | **16.0 %** |
| P(≥ 15) | **44.6 %** |
| P(≥ 14) | 70.8 % |
| Tasks walled off from every worker | 0.11 |
| Energy the optimum actually uses | 35.8k of 48k |

Convergence check: raising restarts from 300 to 5000 on 60 seeds gains **+0.08** mean, so
the true optimum is ≈ 14.2 at most.

**Consequences, which are not negotiable by better code:**

* **16 completed on average is impossible.** A perfect planner with free perfect
  information averages 14.1. Every real scheduler must additionally *pay* for its
  information out of the same 48k energy pool and can only act on what it has seen.
* **"15 completed, stably" is impossible.** The perfect planner clears 15 only 44.6 % of
  the time. No policy can be more reliable than the offline optimum on the same instance.
* The binding constraints are the fleet's **energy** (12000 per robot = 1200 ticks of
  action in a 2000-tick run) and the **clock** (the last task spawns at t=1375, leaving
  625 ticks). Raising energy to 150 % lifts the same router from 12.14 to 14.74; raising it
  to 400 % adds only 0.12 more, so past ~1.5× the run is time-bound, not energy-bound.

### Full accounting of the 16 → 10.58 gap

| Loss | Tasks | Nature |
|---|---|---|
| Walled off from every worker | 0.11 | physically impossible |
| Beyond fleet energy/time even with optimal routing and free perfect information | 1.77 | physically impossible |
| **⇒ ceiling with perfect information** | **14.12** | |
| Our router vs that ceiling (12.65 measured with perfect info, scouts off) | 1.47 | partly unavailable online: the bound still knows *where* future tasks will land when it orders routes |
| Imperfect observation (10.58 real vs 12.65 with perfect info) | 2.07 | the real remaining engineering target |

**Routing ceiling of the current build** (perfect information *and* observation switched
off — `SCHED_T_SMR=2000000000 SCHED_T_PMR=2000000000 ./bench <seed> 16 oracle`, 200 seeds):
**12.65 completed**, 10.0k of 48k worker energy still stranded.

> **Oracle caveat, learned the hard way:** bench's `oracle` mode completes the *known maps*,
> but the scheduler tracks `last_seen` from the real robot views, so under oracle the fleet
> still pays for observation it does not need. A raw `oracle` run therefore *understates* a
> build that observes more. Always disable the scouts (as above) when measuring routing.

### Where the remaining loss is

1. **Observation (≈ 2.1 tasks).** Of the ~3.5 tasks never found, ~1.7 sit on cells never
   observed at all (34.5 of 400 cells are still unknown at t=2000) and ~1.8 are timing
   misses — the cell *was* seen, but before the task spawned there. The budget is ~1.5
   sweeps of the map, so raising discovery past ~12.5 currently costs completions at about
   **1 completion per 3 discoveries** (see the drone pacing curve in §6).
2. **Routing (≤ 1.5 tasks, and less than that is actually available).** See §7.

## 4. Architecture of `schedular.cpp` (navigation map)

`Scheduler::State` holds all planning state; everything is replanned each tick in
`on_info_updated`, which runs in this order:

1. **task snapshot + magnet grid** — free tasks mark `st.magnet` so routed paths get an edge
   discount over them (`TASK_MAGNET`) and workers collect tasks *en route*.
2. **per-robot Dijkstra** — two maps per worker: `dist` (magnet-routed, used for *paths*) and
   `dist_c` (clean, used for *costing*). Mixing them up silently corrupts economics.
3. **fleet plan** — a real multi-vehicle route solve over the known free tasks: seed from last
   tick's routes (stickiness), insertion to place as many tasks as fit within each worker's
   energy and the horizon, then relocate / swap / 2-opt to shorten routes, which frees the
   energy that lets one more task fit. Each worker walks the **first leg of its own route**.
   Two gates survive from the old greedy matcher because they still measure positive: the
   waiting clock (`STARVE_AGE` + `WORKER_TRAVEL_CAP`) on the first leg only, and immediate
   promotion for `sole_server` tasks and the endgame (`ENDGAME_TICKS_DEF`).
4. **`serve_dist`** — cheapest travel energy any still-capable worker could pay to reach each cell.
5. **`build_staleness`** — the observation value model, and the heart of the current design:
   `value(cell) = mass(cell) × serve(cell)` where
   * `mass` = expected **undiscovered** tasks on the cell, from the dispatcher's arrival ramp
     (`spawned_frac`): a never-seen cell carries the initial batch plus everything since; a cell
     seen at tick τ carries only the spawns released after τ. Re-observation is therefore worth
     **exactly zero before the first spawn**, which is what makes the early game pure coverage.
   * `serve` = how cheaply *and how soon* a find there could be completed — the cheapest worker's
     travel energy, ramped to (almost) nothing when the ticks left no longer cover the trip.
6. **drones** — one x-band each, target = argmax of **window value per unit of energy**, committed
   until reached (re-picking every tick oscillates: approaching a target observes away its own
   value). Spending follows a paced line (`DRONE_PACE_T`) so coverage spreads across the run;
   below the line a drone parks and keeps observing as a fixed camera.
7. **workers idle** — same value-per-energy patrol, plus a dispersion bonus for standing where no
   other worker can cheaply reach. Gated by `PATROL_START` and an energy floor that is *released*
   past `PATROL_LATE_T`, because energy still in the tank at t=2000 is a pure loss.

`on_task_reached` is the safety gate: never drones, never a task someone is already working,
never without enough energy to finish, opportunistic takeover only when strictly cheaper.
`idle_action` just converts `next_step` into a direction and refuses to walk into known walls.

Tuning constants are readable from the environment as `SCHED_T_*` (see `load_tunables`) so the
bench harness can sweep them without a rebuild. **Unset environment = the measured defaults.**

## 5. What worked (keep these)

* Exact step-cost Dijkstra (not Manhattan, not uniform cost).
* Split routed/clean distance maps.
* Task magnet on worker paths — free pickups en route.
* Marginal-cost scoring — pushes medium hauls onto energy-rich robots.
* Waiting clock + sole-server protection — stops both over-eager far trips and starvation.
* Solving the routes as an actual VRP rather than scoring tasks one at a time
  (+0.30 completed, and +0.5 on the isolated router).
* **Observation valued as expected *completions*, not staleness** (§4.5). The single biggest change
  this session.
* **Scouts choosing targets by value per unit energy.** The old argmax-value rule would walk 4000
  energy to refresh one 3×3 window.
* **Spending idle worker energy on patrol instead of holding it.** With patrols off, the
  early-coverage profile finishes with 21.9k of 48k worker energy unspent and completes 9.16;
  with them on, 9.70 (and 10.50 after the rest of this session's work).
* Drone commitment (never re-evaluate a goal you are still walking to).
* Steep serve-cost weighting (3.0 → 0.3): observation concentrated where the fleet can reach.

## 6. What was tried and measured to FAIL (do not redo)

Earlier sessions:

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

This session (all paired, 300 seeds, vs the then-current build):

| Attempt | Measured effect |
|---|---|
| **Sparse observation layer on dense task economics** (last session's #1 suggestion) | 8.73 / 14.04 — barely above the sparse profile itself. The late-discovery problem is observation *timing*, not the worker reserve. **Closed.** |
| Same, with worker patrols also off | **7.28 / 13.05, with 29.4k of 48k worker energy stranded.** The decisive measurement: late finds go unserved because they are *unreachable in the time left*, not because the fleet is out of money. |
| Keeping the late rim-ring sweep at all | **−0.97 completed for +2.78 discovered** (same build, observation profile swapped); a find after t≈1700 cannot be walked to |
| Patrolling from t=0, and down to 700 energy | **−2.37 completed** (±0.11) for −0.23 discovered; early patrol energy is exactly what tasks need |
| Patrol from t=500 instead of t=800 | −0.58 completed (±0.07) |
| Value-per-energy ratio with a strong locality offset (`SCOUT_K` 1200) | −0.12 completed, −0.20 discovered |
| Bending worker task routes through cells worth observing (observation magnet, pull 40) | +0.02 ± 0.03 — nothing. Removed. |
| Letting a drone leave its x-band once the band is spent | ±0.00 — a band never actually runs out of value. Removed. |
| Flattening the serve-cost weight (1.4 → 0.9) | −0.04 completed for +0.26 discovered |
| Steepening it past 3.0 → 0.3 (tried 4.5 → 0.2) | −0.09 completed |
| Steeper serve-cost falloff (`SERVE_W_SLOPE` 3500) | −0.23 completed, −0.25 discovered |
| Loosening `WORKER_TRAVEL_CAP` to 5200 | −0.16 completed |
| `STARVE_AGE` 100 (promote waiting tasks sooner) | −0.36 completed, −0.21 discovered |
| Stronger patrol dispersion (3000 vs 1200) | ±0.00 — 1200 already saturates it |
| Bigger up-front drone burst (6000) | −0.07 completed, −0.28 discovered |
| Stronger tour-first preference (0.05) / stickier assignment (0.6) | ±0.00 / −0.03 |
| Residual observation value in the dead zone, at pace 2000 | −0.27 completed for +0.14 discovered |
| Shorter serve-time ramp (`SERVE_RAMP` 80) | ±0.00 |
| More local-search rounds in the fleet plan (12 → 40) | **exactly ±0.00** — the plan is converged at 12 |
| Dropping `WORKER_TRAVEL_CAP` now that a real plan exists | −0.16 completed on the isolated router; the waiting clock still earns its keep |
| Assigning straight from the tour / stronger tour bonus / stickier assignment | ±0.00 each |
| Fixing `window_gain` to count a wheel's cross (5 cells) instead of its square (9) | aggregate-neutral, but it is a correctness fix and re-tuning against the honest value bought +0.09 discovered |
| Drone pacing 1900 | −0.09 completed for +0.27 discovered |

**Drone pacing curve** (`SCHED_T_DPACE`, ticks over which drone fuel is spread) — the cleanest
statement of the completed/discovered trade-off available:

| `DPACE` | Δ completed | Δ discovered |
|---|---|---|
| 0 (fly until empty) | +0.12 | −0.65 |
| 1200 | +0.13 | −0.57 |
| **1700 (current)** | — | — |
| 2000 | −0.18 | +0.35 |
| 2400 | −0.53 | −0.30 |

If a future goal weights *discovered* more heavily, `DPACE` is the dial — but note 2400 loses on
both, so the frontier ends around 2000.

## 7. Suggested attack order for the next session

**First, read §3a.** Any goal above ~14 completed is asking for something the fleet physically
cannot do, and above ~12 is asking to beat the offline optimum's own margin. Set targets against
that table, not against 16.

1. **Observation is now the larger addressable gap (≈ 2.1 tasks).** Two concrete sub-targets:
   * 34.5 of 400 cells are never observed at all. Closing them costs ~7 drone steps (~1.7k
     energy) and there is 4.0k worker + 1.3k drone energy still stranded at t=2000. The value
     model skips them deliberately because they score as expensive to serve — the question is
     whether that judgement is miscalibrated for cells that are merely *currently* far.
   * ~1.8 tasks are timing misses: the cell was seen, but before the task spawned. That needs
     re-observation, which needs capacity, which means cheaper travel.
2. **Routing has ≤ 1.5 left and much of it is not available online.** The 14.12 bound still
   knows where future tasks will land when it orders routes; our 12.65 does not. Before
   investing, build the fair bound: a rolling-horizon version of `bench/plan.cpp` that re-solves
   at each release with only released tasks and commits first legs. If that lands near 12.7,
   routing is finished and everything left is observation.
3. **Do not spend more effort on scalar knobs.** Every knob in §6 has been swept twice; the last
   twenty experiments moved completions by ≤ 0.06 each.

## 8. Bench harness

```bash
bench/build.sh                       # g++ -O2, conio.h shim for Linux
cd bench
./bench <seed> [cap] [-v|oracle]     # one run; CSV: seed,cap,created,discovered,completed,
                                     #   exhausted,end_time,workerE,droneE,droneCellCost
./bench 45 16 -v                     # + per-task spawn/discovery/affordability, miss classifier
                                     #   (never_seen vs timing), robot economics, known map dump
./bench 45 16 oracle                 # perfect information (see the caveat in section 3)
./exp.sh <label> <start> <count> [cap] [mode]   # labelled batch + aggregate, honours SCHED_T_*
./plan <seed> [restarts] [notime|nofore]        # OFFLINE OPTIMUM for that seed -- the ceiling
                                                #   nofore = no clairvoyant pre-positioning
python3 cmp.py <labelA> <labelB>     # PAIRED per-seed diff with standard errors — use this
python3 aggregate.py results.csv 12  # P(metric >= threshold) with Wilson lower bound
BENCH_ENERGY_PCT=200 ./bench 45 16   # diagnostic: is energy the binding constraint?
```

Seeds already spent: tuning 1–400 and 500001–500300; holdouts 10001+, 20001+, 30001+, 40001+,
60001+, 70001+, 900001–900500, 910001–910500, 920001–920500.
**Pick a fresh range for final validation.**

## 9. History of the branch

| Commit | What it is | completed | discovered | seeds |
|---|---|---|---|---|
| `ac88b21` | completion-focused scheduler | 9.16 | 10.35 | 500001–500300 (its profile, forced) |
| `c843d0d` | + discovery layer, regime switch | not re-measured | | |
| `a55c064` | discovery-first tuning + ring triage | 8.80 | 13.96 | 900001–900500 |
| `1744710` | observation valued as expected completions | 10.20 | 12.12 | 500001–500300 |
| `76d898d` | tuning + removal of the unreachable sparse path | 10.27 | 12.25 | 900001–900500 |
| `6455000` | worker routes solved as a real VRP | 10.58 | 12.47 | 910001–910500 |
| **`65165f4`** | **observation reach + endgame retune** | **10.58** | **12.49** | **920001–920500** |

The `ac88b21` row is this session's re-measurement of that build's profile, not the number the
session that produced it reported (9.73 / 10.90 on its own seed set) — different seeds, so compare
rows only within a column's seed range.

The `NUM_MAX_TASKS = 52` experiments existed only to answer an earlier goal ("18+ completions at
90 %", impossible at cap 16 since only 16 tasks ever exist). That goal is closed, and the
regime-switching machinery that served it has been deleted — the scheduler now runs one profile.
`REPORT.md` keeps the full curve and methodology.
