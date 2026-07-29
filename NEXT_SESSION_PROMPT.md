# Prompt for the next session

Copy the block below as the session's opening message (works as a `/goal` argument too).

---

```
Read HANDOFF.md before touching code. Sections 3a and 6 matter most: 3a is the ceiling
analysis and prices any target before you accept it, 6 is ~70 experiments already
measured to fail. Do not redo anything in section 6.

Mission: on the ORIGINAL configuration only (MAP_SIZE 20, 6 robots, NUM_MAX_TASKS 16,
8 initial tasks, TIME_MAX 2000, energy 12000, wall density 20), complete as many of the
16 tasks as possible and discover as many as possible.

Current baseline (1000 fresh holdout seeds, 960001-960500 + 970001-970500):
  completed 12.22 +/- 0.05, discovered 13.63 +/- 0.04

Know the ceiling before you set a target:
  - exact omniscient optimum, verified           14.95 completed
  - current scheduler + free perfect information  14.23 completed / 16.00 discovered
  - current scheduler + UNLIMITED fleet energy    13.61 completed / 15.17 discovered
  - best available given our own discovery        12.93 completed (we get 12.22)
Routing is low-yield -- headroom is in [0, 0.98] -- but NOT proven closed; HANDOFF 3a
explains why the tighter-looking NOFORE bound is not a bound at all. Everything left is
information, worth ~2.3 completions, and it cannot be bought: 200% and 300% fleet energy
give *identical* results because the clock binds, not fuel.

Rules:
- Only schedular.h and schedular.cpp are the deliverable. They must compile as C++14
  (verify: g++ -std=c++14 -w -I bench/shim -fsyntax-only schedular.cpp) and run against
  the pristine main.cpp / simulator.{h,cpp}. Do not edit those three files.
- Every claim needs >= 300 seeds and BOTH metrics reported. Use bench/cmp.py -- the
  paired per-seed diff has ~5x the resolution of comparing two means.
- Screen on 300 seeds, DECIDE on 500+ fresh holdout seeds, and pool two holdout ranges.
  Two of last session's screening winners died on the holdout. A 1-SE screening win is
  nothing.
- For any candidate, run the knob-only control: does it move ALONG the
  completed/discovered frontier, or OUT from it? Only the second kind is progress.
- Pick a fresh seed range for final validation (see HANDOFF section 8 for what is spent).

Report what you measured, what changed, and what it cost. Commit working improvements
with the numbers in the message, and update HANDOFF.md sections 3, 5, 6 and 9 so the
session after you inherits the knowledge.
```

---

## Choosing a `/goal` condition

**Price the target against HANDOFF §3a first.** Last session was given "+2.0 on both
metrics" and that turned out to be unreachable: +2.0 completed (~14.2) is more than the
scheduler achieves with *unlimited* fleet energy (13.61), and essentially all of what free
perfect information delivers (14.23). The session had to spend its ending proving the goal
impossible instead of chasing it.

| Goal text | Note |
|---|---|
| `평균 완료 12.5 이상, 평균 발견 14.0 이상` | realistic next step; ~+0.3 / ~+0.4 from here, both metrics |
| `완료 13개 이상을 50% 확률로` | currently 68.5 % at ≥ 12; genuinely hard |
| `발견 14.5 이상, 완료는 12.2 아래로 내리지 말 것` | discovery-weighted but frontier-safe |

A goal that names **both** metrics is strongly preferred — single-metric goals have twice
produced a build that won its own metric and lost the other. A goal that names a *ceiling*
("get within 0.2 of the bound given our own discovery") is better still, because it stays
meaningful when the absolute number moves.

## Where the remaining loss is (current build, 100 seeds)

| Loss | per run |
|---|---|
| Task on a cell nobody ever observed | 0.72 |
| Task on a cell seen, but only *before* it spawned there | **1.57** |
| Found and affordable, never served | 0.81 |
| Open cells never observed at all | 14.4 of ~320 |

Timing misses are the biggest single bucket. Catching them needs re-observation late in the
run, which is exactly what the fuel does not stretch to.

## One thing to know about single-seed results

`main.cpp` seeds with `srand(time(NULL))`, so the graded run is a random draw. Per-seed
variance is huge — the current build differs from the previous one by anywhere from −6 to +6
completions depending on the seed, and it is *worse* on 34 % of them. Never tune against a
named seed, and when someone reports one, answer with the distribution.
