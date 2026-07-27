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

Current `main` HEAD, original config, **holdout seeds 940001–940500** (never used for anything):

| Metric | Value | Start of this work |
|---|---|---|
| Completed | **12.19** | 8.83 |
| Discovered | 13.34 | 14.11 |
| Conversion | **91 %** | 63 % |
| P(completed ≥ 11) | **89.2 %** | 21.4 % |
| P(completed ≥ 12) | **69.2 %** | 7.2 % |
| Leftover worker energy | 2.4k / 48k | 6.1k |
| Leftover drone energy | **0.2k / 24k** | 2.5k |
Provided placeholder scheduler (random walk): ~7 discovered, **~1 completed**.

## 3a. THE CEILING — read this before accepting any completion target

**A previous version of this file claimed the ceiling was 14.12 and that 16/16 was
impossible. That was wrong and the reasoning was unsound**, so if you have read an older
copy, discard it. `bench/plan.cpp` is insertion + local search: its answer is a solution,
therefore a *lower* bound on the optimum, and a lower bound proves nothing about what
cannot be done. Measured against a real solver it was 1.1 tasks short on average and up
to 5 on individual seeds.

`bench/exact.cpp` solves the instance **exactly**, by subset DP: for each worker a DP over
(subset, last visited) carrying the Pareto frontier of (energy, completion time) — the two
are not interchangeable, because waiting for a release costs time but no energy — then an
exact partition across the four workers. It is a *relaxation* of the simulator (everything
known from t=0, routing free, nothing spent on observation, the simulator's one-tick
arrive-then-start gap ignored), so its answer is a genuine **upper bound**.

**500 seeds:**

| | pre-positioning allowed | no pre-positioning |
|---|---|---|
| Mean optimum | 15.21 | **14.97** |
| P(= 16) | 46.4 % | 34.6 % |
| P(≥ 15) | 81.0 % | 72.6 % |
| Worst seed | 12 | 12 |

So 16/16 *is* reachable on about a third of seeds by a perfect omniscient planner. What is
not reachable is 16 **on average**, and no real scheduler gets that information for free.

### The bound that actually governs us

The number to steer by is not 14.97 — it assumes perfect observation. `./bench <seed> 16
bound` answers the useful question: **given the discovery timeline this scheduler really
produces, what completion count was ever available**, with free optimal routing and no
charge for observation? (`BENCH_BOUND_NOFORE=1` also forbids setting off toward a task
before it is discovered.)

| | 200 seeds |
|---|---|
| We discover | 12.96 |
| We complete | 11.64 |
| **Best available given our own discovery** | **12.22** |
| Routing gap | **0.23** — 130 of 200 seeds already match or beat it |

**Routing is essentially finished.** Everything left is observation — but see the next
section before assuming that means "discover more".

### Discovering more does not help. This is measured, not argued.

Run the same `bound` tool against the *discovery-maximal* build (the pre-session rim-ring
scheduler, which discovers 14.12):

| 200 seeds | current build | discovery-maximal build |
|---|---|---|
| Discovered | 13.12 | **14.12** |
| Completed | 11.99 | 8.72 |
| **Max possible given that discovery timeline** | **12.07** | **10.65** |

The discovery-maximal build finds 1.2 more tasks and its *ceiling* is 1.4 lower than what
we already achieve. Its extra finds land after t≈1700, when no worker can still reach them.
So the completion metric is not limited by how many tasks are discovered but by **when**,
and at this energy budget the two are in direct conflict. A goal expressed as "discover
more" is a goal to make completions worse.

### Full accounting of the 16 → 11.69 gap

| Loss | Tasks | Nature |
|---|---|---|
| Walled off from every worker | 0.11 | impossible |
| Beyond fleet energy/time even for the exact omniscient optimum | 0.92 | impossible |
| **⇒ exact ceiling with perfect information** | **14.97** | |
| Tasks we never discover in time to serve | ~2.9 | **the remaining target** |
| Routing, measured against our own discovery timeline | 0.23 | closed |

## 3b. Why the shape of the solution is what it is

A robot carries 6 energy per tick of the run, so it can act for at most **1200 of the 2000
ticks** no matter what it does. That single fact drives the whole design: acting early
spends a fixed budget on the worst information the run will ever have. So the workers
**hold until t=800 and then spend everything**, which is exactly their energy budget, with
the map as complete as it is going to get. Before t=800 a worker serves only a task it is
standing on or one nobody else could ever reach.

**t=800 is not a constant.** It is exactly where a full worker's 12000 energy stops fitting
in the 1200 ticks it has left, so the rule in the code is per worker and parameter-free:
commit once `energy >= 10 * ticks_left`. Testing that crossover with a multiplier confirms
it is the right *quantity* and not a fitted number — 0.9 costs 1.5 completions and 1.1
costs 2.4, a sharp peak at exactly 1.0. It is also monotone: energy and the threshold both
fall at 10 per tick, so a worker cannot fall back through the gate once it has committed.
The patrol gate uses the same crossover and, for a worker that has held, reproduces t=800
exactly.

Every attempt to make the workers do *anything* before that moment loses: patrolling from
t=400 costs 2.67 completions, from t=200 costs 4.27, a 500-energy sip of early scouting
costs 0.19, and giving up a whole worker as a scout costs 1.6 (and lowers discovery too).

## 4. Architecture of `schedular.cpp` (navigation map)

`Scheduler::State` holds all planning state; everything is replanned each tick in
`on_info_updated`, which runs in this order:

1. **task snapshot + magnet grid** — free tasks mark `st.magnet` so routed paths get an edge
   discount over them (`TASK_MAGNET`) and workers collect tasks *en route*.
2. **per-robot Dijkstra** — two maps per worker: `dist` (magnet-routed, used for *paths*) and
   `dist_c` (clean, used for *costing*). Mixing them up silently corrupts economics.
3. **fleet plan** — solved **exactly** whenever at most `EXACT_MAX` (16) free tasks are known,
   which is always: per worker a DP over (subset, last visited) carrying the Pareto frontier of
   (energy, completion time), then an exact partition across workers maximising the count and
   breaking ties on energy. Local search (insertion + relocate/swap/2-opt, seeded from last
   tick's routes) remains as the fallback. Each worker walks the **first leg of its own route**.
   The first leg is gated by `WORKER_TRAVEL_CAP` (0) until the endgame opens at
   `ENDGAME_TICKS_DEF` (1200 ticks before the end, i.e. t=800) — see §3b for why.
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
* Solving the routes as an actual VRP rather than scoring tasks one at a time, and then
  solving that VRP **exactly** rather than by local search (+0.32 on the holdout).
* Holding the workers until they can spend their whole budget (§3b). The single largest
  change in this work: +0.81 completed on the holdout.
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

Second round (all paired, 300 seeds unless noted):

| Attempt | Measured effect |
|---|---|
| Holding worker energy back for tasks that have not spawned yet | −0.06 at 2000, −0.23 at 4000. The right mechanism is the travel cap, not a reserve. |
| Weighting routes by where they leave a worker, valued against a uniformly random future spawn | −0.18 at 400, −0.31 at 1000 |
| Keeping a nearly-reached target first (commitment lock) | −0.016 ± 0.010 over 500 seeds |
| Bending worker task routes through cells worth observing | +0.02 at pull 40; **−0.18 completed for +0.16 discovered** at 150 |
| Extra mass on never-seen cells (×2.5) | −0.11 completed |
| Any worker movement before t=800 | −2.67 at patrol-start 400, −4.27 at 200, −0.19 for even a 500-energy sip |
| Patrol start 700 or 1000 instead of 800 | −0.45 / −0.18 |
| Endgame window 1050, 1350, 1500, 1800 | −0.09 / −0.23 / −0.19 / −0.43 — 1200 is a genuine peak |
| More local-search rounds (12 → 40) | exactly ±0.00; irrelevant now that the plan is exact |
| `EXACT_MAX` above 14 | exactly ±0.00 — the free-task count never exceeds 14 |
| Re-sweeping the observation weights after the strategy change (`SERVE_W_HI` 4.0, `SERVE_W_LO` 0.4, `SERVE_RAMP` 450) | −0.10 / ±0.00 / −0.02 |
| Drone pacing 1200 / 2000 / 1550 / 1850 after the strategy change | −0.59 disc / −0.22 comp / +0.07 comp for −0.18 disc / −0.18 comp |
| Drone up-front burst 0 / 6000 | −0.06 comp for +0.13 disc / +0.05 comp for −0.21 disc — same trade either way, neither established |
| Drone pacing 1600 / 1800 after the tail spend-down | ±0.00 / −0.06 — 1700 survives every re-check |
| **Two-phase drone pacing keyed to the workers' commit moment** (compress the first pass into t<800, since within that window it makes no difference when a cell is first seen) | −0.18 to −0.30 discovered at every split tried (50/60/70 %), completions flat. The plausible-sounding argument is simply wrong: the value-greedy already orders first coverage ahead of re-coverage, and forcing more of the budget early just spends it on cells it would have reached anyway. |
| Lowering the energy at which a worker stops counting as a possible server (1500 → 600, or → 2500) | exactly ±0.00 — the reach cap already dominates it |
| Residual observation value in the dead zone at 0.15 or 0.5 | ±0.00 either way |
| Worker patrol floor 800, late release at t=1000 | ±0.00 |
| Dropping the sole-server promotion | ±0.00 |
| Plan finishing margin 10 or 20 ticks instead of 0 | −0.05 / −0.12 |
| Giving up a whole worker as a full-time scout | **−1.6 completed at one, −3.9 at two — and discovery *falls* too** (12.45 vs 13.02). Trading service capacity for observation loses at every scale tested. |
| More local-search rounds in the fleet plan (12 → 40) | **exactly ±0.00** — the plan is converged at 12 |
| Dropping `WORKER_TRAVEL_CAP` now that a real plan exists | −0.16 completed on the isolated router; the waiting clock still earns its keep |
| Assigning straight from the tour / stronger tour bonus / stickier assignment | ±0.00 each |
| Fixing `window_gain` to count a wheel's cross (5 cells) instead of its square (9) | aggregate-neutral, but it is a correctness fix and re-tuning against the honest value bought +0.09 discovered |
| Drone pacing 1900 | −0.09 completed for +0.27 discovered |
| Removing the locality penalty on observation value entirely (`SERVE_W_LO` 1.2) | −0.03 completed, no discovery gain |
| Less locality bias in the scout ratio (`SCOUT_K` 250) | −0.07 completed, −0.07 discovered |
| Smaller up-front drone burst (1500) | −0.07 completed for +0.12 discovered |
| Lower drone move threshold (`SCOUT_MIN_RATIO` 400) | **exactly ±0.00** — the threshold never binds |

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

**Read §3a and §3b first.** The exact ceiling is 14.97 with free perfect information; the bound
that governs us, given the discovery we actually achieve, is **12.07**, and we are at 11.69.
Set targets against those, and expect the next whole task to be hard.

0. **The pattern that keeps paying is "find a reserve guarding a future that no longer
   exists, and spend it".** The drone camera floor guarded against missing a spawn — worth
   nothing once the dispatcher is done, and collapsing it was worth +0.22 completed *and*
   +0.23 discovered at once, which almost nothing else here does. The worker commit moment
   is the same idea in reverse (hold while the energy is scarce, spend when it could not
   all be used anyway). If you find another such reserve, it is probably the best-value
   thing on this list.
1. **Observation is otherwise the only meaningful lever** — the routing gap is 0.43 and 92 of 200
   seeds are already optimal. We discover 13.0 of 16. Of the ~3 we miss, ~1.1 sit on cells
   nobody ever observed (22.8 open-ground cells per run are never seen) and the rest are timing
   misses, where the cell was seen but before the task spawned there.
2. **The observation budget is the constraint, and it is now almost entirely the drones'** —
   the workers cannot spare a single unit before t=800 without losing more than it buys (§6).
   Two drones carry 24k, about 1.5 sweeps of the map. Any real gain has to come from covering
   more cells per unit of drone energy, not from re-weighting which cells they prefer: the
   weights have been swept twice and are at a local optimum.
   The one structural idea not yet tried is an explicit boustrophedon lane sweep timed to the
   spawn schedule, instead of the value-per-energy greedy. Earlier sessions rejected *fixed
   lattice* sweeps, but never a lane sweep driven by the current value model.
3. **Do not re-sweep scalar knobs.** Everything in §6 has now been swept three times.

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
./plan <seed> [restarts] [notime|nofore]        # heuristic offline plan -- a LOWER bound only
./exact <seed> [nofore]                         # EXACT optimum by subset DP -- the real ceiling
./bench <seed> 16 bound                         # best available GIVEN our own discovery times
                                                #   (BENCH_BOUND_NOFORE=1 forbids pre-positioning)
python3 cmp.py <labelA> <labelB>     # PAIRED per-seed diff with standard errors — use this
python3 aggregate.py results.csv 12  # P(metric >= threshold) with Wilson lower bound
BENCH_ENERGY_PCT=200 ./bench 45 16   # diagnostic: is energy the binding constraint?
```

Seeds already spent: tuning 1–400 and 500001–500300; holdouts 10001+, 20001+, 30001+, 40001+,
60001+, 70001+, 900001–900500, 910001–910500, 920001–920500, 930001–930500,
940001–940500.
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
| `65165f4` | observation reach + endgame retune | 10.58 | 12.49 | 920001–920500 |
| `58ad564` | exact fleet plan added (but shipped switched off) | — | — | |
| `1bfbea9` | hold the workers, then go all in | 11.44 | 12.93 | 930001–930500 |
| `561cbea` | exact plan actually enabled | 11.69 | 12.99 | 940001–940500 |
| `5b52680` | commit moment derived, not tuned | 11.95 | 13.00 | 940001–940500 |
| **HEAD** | **spend reserves that guard a future that is over** | **12.18** | **13.32** | **940001–940500** |

The `ac88b21` row is this session's re-measurement of that build's profile, not the number the
session that produced it reported (9.73 / 10.90 on its own seed set) — different seeds, so compare
rows only within a column's seed range.

The `NUM_MAX_TASKS = 52` experiments existed only to answer an earlier goal ("18+ completions at
90 %", impossible at cap 16 since only 16 tasks ever exist). That goal is closed, and the
regime-switching machinery that served it has been deleted — the scheduler now runs one profile.
`REPORT.md` keeps the full curve and methodology.
