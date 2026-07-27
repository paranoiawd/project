# Prompt for the next session

Copy the block below as the session's opening message (works as a `/goal` argument too).

---

```
Read HANDOFF.md first — it records the simulator mechanics already derived, the current
measured baseline, what works, and what has already been tried and failed. Do not redo
anything in the "measured to FAIL" table.

Mission: on the ORIGINAL configuration only (MAP_SIZE 20, 6 robots, NUM_MAX_TASKS 16,
8 initial tasks, TIME_MAX 2000, energy 12000, wall density 20), make the scheduler
discover as many of the 16 tasks as possible and complete as many as possible.
Completed is the primary metric; discovered is the enabler. The ideal run is 16/16.

Current baseline to beat (500 fresh seeds): 14.03 discovered / 8.72 completed with the
discovery profile, 10.90 discovered / 9.73 completed with the completion profile.
Perfect-information oracle completes only 12.22/16, so routing — not just observation —
is leaving tasks on the table.

Rules:
- Only schedular.h and schedular.cpp are the deliverable; they must compile as C++14 and
  run against the pristine main.cpp / simulator.{h,cpp}. Do not edit main.cpp.
- Every claim must be backed by a measurement over >= 300 seeds, and always report BOTH
  discovered and completed — optimizing one alone has already backfired.
- Validate final numbers on holdout seeds not used for tuning (10001+, 20001+, ... are
  already spent; pick a fresh range).
- Use bench/ (see HANDOFF.md section 8). The oracle mode isolates routing from observation.

Work in this order unless evidence says otherwise:
1. Try the sparse observation layer on the dense task economics (they are currently coupled
   through st.regime) — targets the 62% -> 89% conversion collapse.
2. Use oracle mode to find why ~3.8 tasks die even under perfect information; fix that
   routing/energy allocation.
3. Only then chase the last ~2 discoveries, preferring observation that happens along task
   routes over dedicated patrols.

Report progress concisely as you go: what you measured, what changed, what it cost.
Commit working improvements with the numbers in the message, and update HANDOFF.md's
baseline table and FAIL table so the session after you inherits the knowledge.
```

---

## Suggested `/goal` conditions

Pick one; they are ordered by ambition. Current baseline is ~8.7–9.7 completed.

| Goal text | Note |
|---|---|
| `원본 구성에서 평균 완료 11개 이상, 평균 발견 14개 이상` | realistic next step; both metrics move |
| `원본 구성에서 완료 12개 이상을 50% 확률로` | ≈ the current perfect-information ceiling |
| `원본 구성에서 16개 발견 16개 완료를 달성` | the ultimate target; likely above the physical ceiling — expect the session to report a bound instead |

A goal that names **both** metrics is strongly preferred: the single-metric goals in the
previous session each produced a build that won its own metric and lost the other.
