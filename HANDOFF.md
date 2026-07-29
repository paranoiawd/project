# HANDOFF — state of the work, for the next session

**Mission from here on: on the ORIGINAL configuration (16 tasks), discover as many tasks as
possible and complete as many as possible. The ideal run is 16 discovered / 16 completed.
Completed is the primary metric; discovered is the enabler.**

Read this file before touching code. It records what is already known, what already works, and —
importantly — what has already been tried and measured to *fail*, so none of it is redone.

---

## 0. Start here (30 seconds of orientation)

**State:** completed **12.22**, discovered **13.63** (1000 fresh holdout seeds). The provided
placeholder scheduler completes ~1.

**Read in this order.** §3a prices any target you are about to accept — read it before agreeing
to a number. §6 is ~70 experiments already measured to fail. §4 is the code map.

**The three facts that determine what is worth trying:**

1. **Routing has at most ~1.0 of headroom and probably a few tenths — but it is NOT proven
   closed**, and earlier versions of this file over-claimed that. See §3a: the only *valid*
   bound given our own discovery timeline is 12.93 against our 11.95, and the tighter-looking
   `NOFORE` variant (12.21) is not a bound at all. The independent check is the oracle: with
   free perfect information the current scheduler completes 14.23 against a verified exact
   optimum of 14.95, so execution under good information is within 0.72 of an unattainable
   ideal. Treat routing as low-yield, not finished.
2. **Everything left is information**, worth ~2.3 completions — far more than routing.
3. **Information cannot be bought.** At 200 % *and* 300 % fleet energy the scheduler returns
   *identical* numbers (13.61 / 15.17). Past that point the clock binds, not fuel — a drone step
   costs 18–30 ticks, a full map sweep needs ~79 steps, and two drones can afford ~1.25 sweeps
   in the whole run.

**So the only thing that has ever moved this number is making each step of the existing fuel
count for more.** That is what the last change did, and the pattern generalises: *look for
something the scheduler pays for but does not credit itself for.* The scout was being charged
for a whole trip and credited only with the window it ended on.

**How to not waste the session:** screen on 300 seeds, decide on 500+ *fresh* seeds, pool two
holdout ranges. Two of last session's screening winners (`SCOUT_K`=1500, `SCOUT_RECOMMIT`=150)
were >1 SE wins on the tuning set and died on the holdout. And for every candidate run the
knob-only control — a change that moves *along* the completed/discovered frontier is not
progress, only one that moves *out* is.

**What changed last session** (see §5, §6, and §9 for the numbers):
* `bench/exact.cpp` was independently verified rather than trusted — it is sound (§3a).
* Scout targets are now scored by everything the trip sees, not the destination window.
* Equal-cost paths are steered by observation value (free by construction).
* The serve weighting is off, which is only correct because of the two changes above.
* The stale ceiling numbers in this file were re-measured; several were badly wrong.

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

Current `main` HEAD, original config, **1000 fresh holdout seeds** (960001–960500 and
970001–970500, neither used for any tuning), paired against the previous HEAD:

| Metric | previous HEAD | **current HEAD** |
|---|---|---|
| Completed | 12.04 | **12.22** (+0.181 ± 0.052) |
| Discovered | 13.24 | **13.63** (+0.387 ± 0.053) |
| P(completed ≥ 12) | 66.9 % | **68.5 %** |
| P(discovered ≥ 14) | 46.6 % | **57.5 %** |
| Open cells never observed | 20.1 | **14.4** |
| Leftover worker energy | 2.4k / 48k | 2.5k / 48k |
| Leftover drone energy | 0.3k / 24k | 0.4k / 24k |

Miss breakdown, same build: coverage holes 0.99 → **0.72** per run, timing misses 1.89 →
**1.57**, initial-8 found 7.49 → **7.64**, late-8 found 5.63 → **6.07**.
Provided placeholder scheduler (random walk): ~7 discovered, **~1 completed**.

## 3a. THE CEILING — read this before accepting any completion target

### `bench/exact.cpp` has now been independently verified — treat it as sound

It was re-audited from scratch rather than trusted. `bench/verify_exact.cpp` checks it three
ways, sharing no code with the DP it is checking, and **100/100 seeds pass**:

| Check | What it does | Result |
|---|---|---|
| **A1 algorithm** | every servable subset up to size 6, brute-forced over *all permutations* and evaluated directly, vs the Pareto DP's answer | **2.75 M subsets, 0 mismatches** |
| **A2 partition** | the layered subset-sum DP vs an independent recursive partition search | 0 mismatches |
| **B model** | concrete routes replayed in the **real simulator**, driving a real `ROBOT` through `start_moving`/`move`/`start_working`/`work`, comparing predicted energy and completion tick against reality | 263 routes, **0 mismatches** |
| falsification | 300 real runs vs the bound on the same seeds | 0 violations |

The mechanics were also re-derived from `simulator.{h,cpp}` and every one matches: step cost
`ceil((floor(cost(u)/2)+cost(v))/10)` ticks at 10 energy/tick (each step re-ceils — arrival
overshoot is discarded), work `10*max(1,ceil(cost/10))`, HOLD free, and `t <= TIME_MAX`
exactly right (the last usable tick is 1999, and `main.cpp` re-reads `status` by reference so
no tick is lost between arriving and starting).

One caveat found and worth knowing: `exact` runs the dispatcher with no robots moving, but
`get_random_empty_coord` rejects robot-occupied cells, so the *positions* of tasks 8–15 in a
real run diverge from `exact`'s instance. It does not affect the bound as a distributional
statement, and it is why check B replays initial tasks only.

### What the ceiling actually is (measured this session, current build)

| Regime | completed | discovered |
|---|---|---|
| current build, real energy, real information | 12.22 | 13.63 |
| best available **given our own discovery timeline** (valid bound) | 12.93 | — |
| current scheduler + **free perfect information** | **14.23** | 16.00 |
| current scheduler + **unlimited fleet energy** (200 % and 300 % are identical) | **13.61** | 15.17 |
| exact omniscient optimum, no pre-positioning (verified) | **14.95** | 16 |

Read those rows together, because they decide what any completion target is worth:

* **Routing is low-yield but not proven closed** — headroom is in [0, 0.98]; see the bound
  discussion below, which corrects an over-claim in earlier versions of this file.
* **The whole remaining gap is information**, and it is worth 2.0–2.3 completions.
* **Energy stops binding before the target is reached.** At 200 % fleet energy the scheduler
  gets 13.61/15.17 and at 300 % it gets *exactly the same numbers* — beyond that point the
  limit is the clock, not fuel: a drone step takes 18–30 ticks, so no amount of energy buys
  enough re-sweeps to catch the late spawns.

So a goal of "+2.0 completed" (→ ~14.2) asks for more than **unlimited fuel** delivers (13.61)
and essentially all of what **free perfect information** delivers (14.23). It is not reachable.
"+2.0 discovered" (→ ~15.6) is above every configuration ever measured at real energy (the
discovery-maximal builds top out near 13.9–14.1) and needs roughly 2× fleet energy (15.17).

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

### The bound that actually governs us — and how to read it correctly

`./bench <seed> 16 bound` asks: **given the discovery timeline this scheduler really produces,
what completion count was ever available**, with free optimal routing and no charge for
observation? There are two variants and **neither is the number you want on its own**:

| 200 seeds, current build | free (default) | `BENCH_BOUND_NOFORE=1` |
|---|---|---|
| We complete | 11.95 | 11.95 |
| Bound reports | **12.93** | 12.21 |
| Real runs that **exceed** it | **0 / 200** | **39 / 200 (20 %)** |
| Status | **valid upper bound**, but loose | **not a bound** |

**`NOFORE` is not an upper bound, and this file previously said it was.** It forbids a worker
from setting off toward a task before that task is discovered — but a real worker is often
*already en route* through a region when a discovery lands there, and picks it up. That is
legal, it happens, and it is why 20 % of real runs beat the "bound". A `NOFORE` gap of 0.265
does **not** mean routing is 0.265 from optimal.

The default variant *is* a valid relaxation (nothing is forbidden that reality allows), but it
is loose in the other direction: it lets a worker pre-position clairvoyantly toward a task it
cannot know about, which is worth ~0.7.

**So the honest statement is: routing headroom lies in [0, 0.98], and the two variants bracket
it.** If you want to attack routing, first build a bound that is tight from both sides —
neither of these is.

The independent evidence that routing is nonetheless low-yield: give the *current* scheduler
free perfect information and it completes **14.23** against a verified exact optimum of 14.95.
Whatever the remaining loss is, most of it is not the router.

### Discovery cannot simply be bought — the clock binds before fuel does

`BENCH_ENERGY_PCT` scales every robot's energy. The result is the most important single fact
about this problem:

| fleet energy | completed | discovered |
|---|---|---|
| 100 % (the real rules) | 12.08 | 13.46 |
| 150 % | 13.56 | 14.71 |
| **200 %** | **13.61** | **15.17** |
| **300 %** | **13.61** | **15.17** |

200 % and 300 % are *identical numbers*, not merely close. Past that point energy is not the
constraint at all — the clock is. A drone step costs 18–30 ticks, one full map sweep needs
~79 steps, and two drones can afford about **1.25 sweeps** in 2000 ticks. There is no second
sweep to be had at any price, which is why the late spawns go unfound.

Use this table to price any completion target before accepting it. Anything above 13.6 is
asking for more than infinite fuel delivers.

### "Discover more" is a trap — but only one kind of it

The old rim-ring scheduler discovered 14.12 and completed 8.72; its extra finds landed after
t≈1700 when no worker could still reach them. Measured with the `bound` tool, its *ceiling*
was 1.4 lower than what the current build already achieves.

So: **buying discovery by spending fuel later is a trap** — it moves along the frontier and
costs completions. `DRONE_PACE_T` is that dial and it is still a straight trade (§6).

But that is not the same as "discovery cannot rise". **Making each step of the existing fuel
count for more raises both**, and that is exactly what this session's change did: +0.18
completed *and* +0.39 discovered together, with the control run confirming the knob alone
would have been −0.07 for +0.28. When you find a candidate change, check which of the two it
is — a frontier move or a frontier expansion — by running the knob-only control.

### Full accounting of the 16 → 12.22 gap

| Loss | Tasks | Nature |
|---|---|---|
| Walled off from every worker | 0.11 | impossible |
| Beyond fleet energy/time even for the exact omniscient optimum | 0.92 | impossible |
| **⇒ exact ceiling with perfect information** | **14.97** | |
| Tasks never discovered in time to serve | ~2.5 | **the remaining target** |
| Routing, against the only valid bound given our discovery | ≤ 0.98 | low-yield, not closed |

Of the tasks missed, measured over 100 seeds on the current build: **0.72 per run sit on cells
nobody ever observed** (14.4 open cells per run are never seen) and **1.57 are timing misses**,
where the cell was seen but only before the task spawned there. Timing is the bigger half.

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
   * `serve` = how cheaply *and how soon* a find there could be completed. **This factor is
     now neutral** (`SERVE_W_HI` = `SERVE_W_LO` = `SERVE_W_DEAD` = 1.0) and that is measured,
     not an oversight — see §5. The knobs are still live for re-sweeping.
6. **drones** — one x-band each, target = argmax of **value per unit of energy over the whole
   trip** (`path_gain`: a cumulative walk over the shortest-path tree crediting every cell the
   route brings into view for the first time, not just the destination window). Committed until
   reached — re-picking every tick oscillates, because approaching a target observes away its
   own value. Spending follows a paced line (`DRONE_PACE_T`) so coverage spreads across the
   run; below the line a drone parks and keeps observing as a fixed camera.
6b. **path steering** — `dijkstra` takes an optional direction-indexed grid of observation
   value (`build_edge_val`, built from row/column prefix sums so each lookup is O(1)). It never
   changes *which* cells are cheapest to reach, only which of the equally cheap paths is taken,
   so it is free. Worker service routes get it too.
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
* ~~Steep serve-cost weighting~~ — **superseded, see below.**
* **Scoring a scout trip by everything it sees on the way, not by the window it ends on**
  (`SCOUT_PATHVAL`). The old rule costed the whole trip but only credited the destination,
  which is the wrong economics for a fuel-limited scout: two targets with identical end
  windows can differ by a whole sweep's worth of coverage. Computed as a cumulative walk over
  the shortest-path tree, so it is O(cells) per scout per tick.
* **Steering equal-cost paths by observation** (`PATH_TIEBREAK`). Shortest paths in this grid
  are rarely unique and the tie was settled by queue order. Picking the better-observing one
  among *exactly equally cheap* paths costs zero energy by construction.
* **Turning the serve weighting off, but only after the two above.** Flattening it was worth
  −0.07 completed on the old value model and **+0.23 completed / +0.39 discovered** on the new
  one. It was double-pricing distance: the energy denominator already charges for a long trip,
  and weighting it again suppressed exactly the sweeps that pay.

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
| Stronger patrol dispersion (3000 vs 1200) | ±0.00 — 1200 already saturates it; retested after the strategy change, still ±0.00, and turning it off costs 0.05 |
| Bigger up-front drone burst (6000) | −0.07 completed, −0.28 discovered |
| Stronger tour-first preference (0.05) / stickier assignment (0.6) | ±0.00 / −0.03 |
| Residual observation value in the dead zone, at pace 2000 | −0.27 completed for +0.14 discovered |
| Shorter serve-time ramp (`SERVE_RAMP` 80) | ±0.00 |

Second round (all paired, 300 seeds unless noted):

| Attempt | Measured effect |
|---|---|
| Holding worker energy back for tasks that have not spawned yet | −0.06 at 2000, −0.23 at 4000. **Retested after the hold-then-commit change**, where the argument for it is much stronger: it looked like +0.04 at 1000 on the tuning set (1.5 SE) and did not replicate on the holdout (−0.01 ± 0.02). Not shipped. The right mechanism is the commit crossover, not a reserve. |
| Weighting routes by where they leave a worker, valued against a uniformly random future spawn | −0.18 at 400, −0.31 at 1000 |
| Keeping a nearly-reached target first (commitment lock) | −0.016 ± 0.010 over 500 seeds |
| Bending worker task routes through cells worth observing | +0.02 at pull 40. **Retested after the hold-then-commit change**, where worker travel is concentrated and precious: −0.02 at 60, −0.29 at 150, −0.59 at 300. Worse than before, not better. Code removed. |
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

This session (path-integrated observation value). Screened on seeds 1–300, decided on 500-seed
holdouts — note how many screening wins did not survive that:

| Attempt | Measured effect |
|---|---|
| `SCOUT_K` 1500 (bigger locality offset, re-tuned for the new value scale) | +0.07 discovered on the tuning seeds, **−0.17 completed and −0.16 discovered on the holdout**. Did not replicate; kept at 500. A reminder that a 1-SE screening win is nothing. |
| `SCOUT_K` 0 (pure rate, no offset) | −0.06 completed, ±0.00 discovered |
| Drone pacing 1800 / 2000 / 2100 under the new model | +0.05 comp for −0.13 disc / −0.08 comp / −0.16 comp for +0.40 disc — 1900 is the joint peak |
| Partial serve weighting (`SERVE_W_HI` 1.5, 2.0, 2.5 with `LO`=`DEAD`=1.0) | all worse than fully flat once pacing is at 1900; 2.0 at pace 1700 was the best partial (+0.14/+0.21) but lost to flat+1900 on the holdout (+0.09/+0.04) |
| **Abandoning a committed drone target when something is worth ≥1.5× more** (`SCOUT_RECOMMIT` 150) | +0.09/+0.13 on one holdout, **−0.08/+0.01 on the other**; pooled +0.007 completed, +0.071 ± 0.039 discovered. Mechanism is kept and knob-selectable, **shipped off** — it does not clear the bar. |
| Flattening the serve weight *without* the path-integrated value | −0.07 completed for +0.28 discovered — a move *along* the frontier, not out. This is the control that shows the code change, not the knob, is doing the work. |
| **Letting a drone leave its x-band when the other side pays a better *rate*** (not merely "when its own band is spent", which is the version §6 already killed) | −0.04 completed and −0.13 discovered at a 1.5× margin; at 3× the margin is never met, so it is an exact no-op. The bands are not a crude approximation to be relaxed — they are what stops the two scouts duplicating each other's coverage, and rate-based release re-derives the same answer. |

**Drone pacing curve** (`SCHED_T_DPACE`, ticks over which drone fuel is spread) — the cleanest
statement of the completed/discovered trade-off available. Measured under the *old* value
model; the joint peak has since moved from 1700 to **1900** (§6):

| `DPACE` | Δ completed | Δ discovered |
|---|---|---|
| 0 (fly until empty) | +0.12 | −0.65 |
| 1200 | +0.13 | −0.57 |
| 1700 | — | — |
| 2000 | −0.18 | +0.35 |
| 2400 | −0.53 | −0.30 |

## 7. Suggested attack order for the next session

**A literature survey with ranked method candidates now exists in [`EXPLORATION.md`](EXPLORATION.md)**
(2026-07-29 session): what the DVR/IPP/anticipatory-dispatch fields use for exactly this problem
shape, which of it §6 already killed, and a 5-step experiment order starting with a cheap upper-bound
check on the scout tour before any implementation.

**Read §3a first — it has been rewritten and the old numbers in it were stale.** The verified
omniscient optimum is 14.95; the only *valid* bound given the discovery we actually achieve is
12.93 and we are at 12.22. Set targets against those, and read §3a on why the tighter-looking
`NOFORE` number is not a bound.

0. **Routing is low-yield, but do not repeat this file's old claim that it is "closed".**
   The rigorous headroom is [0, 0.98] (§3a) — the tight-looking 0.265 came from a `NOFORE`
   bound that real runs beat on 20 % of seeds, so it was never a bound. What *is* solid: with
   free perfect information the current scheduler completes 14.23 against an exact optimum of
   14.95. If you want the routing tenths, the first job is a bound that is actually tight —
   one that lets a worker be en route when a discovery lands, but not walk toward a task it
   cannot know about.
1. **Everything left is information, and it is worth 2.0 completions.** Perfect information is
   +2.28 completed and +2.89 discovered on the same seeds. Nothing else on this list is worth
   a tenth of that.
2. **But observation cannot simply be bought.** At 200 % fleet energy the scheduler reaches
   13.61/15.17 and at 300 % it reaches *exactly the same numbers*. Past that point the binding
   constraint is the clock: a drone step costs 18–30 ticks, the map needs ~79 steps to sweep
   once, and the two drones together can afford about 1.25 sweeps in 2000 ticks. There is no
   second sweep to be had, at any price.
3. **So the only thing that ever moves this is making each step count for more.** That is what
   worked this session: the scout's value model was costing whole trips while crediting only
   their last window (§5). The same question is worth asking again elsewhere — *where else is
   something priced but not credited?*
4. **Do not re-sweep scalar knobs, and do not trust a screening win.** Of this session's
   experiments, `SCOUT_K`=1500 and `SCOUT_RECOMMIT`=150 both won on 300 tuning seeds and both
   died on 500-seed holdouts. Screen at 300, decide at 500+, and pool both holdouts.

## 8. Bench harness

```bash
bench/build.sh                       # g++ -O2, conio.h shim for Linux (bench only)
bench/build.sh all                   # + exact, verify_exact, plan
cd bench
./bench <seed> [cap] [-v|oracle]     # one run; CSV: seed,cap,created,discovered,completed,
                                     #   exhausted,end_time,workerE,droneE,droneCellCost
./bench 45 16 -v                     # + per-task spawn/discovery/affordability, miss classifier
                                     #   (never_seen vs timing), robot economics, known map dump
./bench 45 16 oracle                 # perfect information (see the caveat in section 3)
./exp.sh <label> <start> <count> [cap] [mode]   # labelled batch + aggregate, honours SCHED_T_*
./plan <seed> [restarts] [notime|nofore]        # heuristic offline plan -- a LOWER bound only
./exact <seed> [nofore]                         # EXACT optimum by subset DP -- the real ceiling
./verify_exact <seed> [max_subset=6]            # VERIFIES ./exact: brute-force permutations,
                                                #   independent partition solver, and a replay
                                                #   of real routes in the real simulator
./bench <seed> 16 bound                         # best available GIVEN our own discovery times.
                                                #   This default form is the VALID one (loose).
                                                #   BENCH_BOUND_NOFORE=1 looks tighter but is
                                                #   NOT a bound -- real runs beat it 20% of the
                                                #   time.  See 3a before quoting either.
python3 cmp.py <labelA> <labelB>     # PAIRED per-seed diff with standard errors — use this
python3 aggregate.py results.csv 12  # P(metric >= threshold) with Wilson lower bound
BENCH_ENERGY_PCT=200 ./bench 45 16   # diagnostic: is energy the binding constraint?
```

Results now land in `bench/out/` (git-ignored) and `cmp.py` reads from there, so the harness no
longer depends on a session-specific scratch path. Override with `BENCH_OUT=<dir>`.

Seeds already spent: tuning 1–400 and 500001–500300; holdouts 10001+, 20001+, 30001+, 40001+,
60001+, 70001+, 900001–900500, 910001–910500, 920001–920500, 930001–930500,
940001–940500, 950001–950300, 960001–960500, 970001–970500.
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
| `14643ed` | spend reserves that guard a future that is over | 12.18 | 13.32 | 940001–940500 |
| | *(same build re-measured on the seeds below)* | 12.04 | 13.24 | 960001+970001, n=1000 |
| **HEAD** | **pay a scout for what it sees on the way** | **12.22** | **13.63** | **960001+970001, n=1000** |

The `ac88b21` row is this session's re-measurement of that build's profile, not the number the
session that produced it reported (9.73 / 10.90 on its own seed set) — different seeds, so compare
rows only within a column's seed range.

The `NUM_MAX_TASKS = 52` experiments existed only to answer an earlier goal ("18+ completions at
90 %", impossible at cap 16 since only 16 tasks ever exist). That goal is closed, and the
regime-switching machinery that served it has been deleted — the scheduler now runs one profile.
`REPORT.md` keeps the full curve and methodology.
