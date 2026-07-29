# EXPLORATION — 방법 탐색: 문헌 조사와 후보 랭킹 (2026-07-29 세션)

이 문서는 "지금 접근에 갇히지 말고 다른 분야에서 뭘 쓰는지 보라"는 지시로 수행한
문헌 조사의 결과다. 코드 변경은 없다. 목적은 다음 세션(들)이 바로 실험에 들어갈 수
있도록 (a) 이 문제의 학술적 좌표를 찍고, (b) 분야별 대표 방법을 §6(실패 실험 ~70개)과
대조해 걸러내고, (c) 실행 가능한 후보를 예상 수익·비용·리스크와 함께 랭킹하는 것.

HANDOFF.md가 여전히 단일 진실 원천이다. 여기 있는 어떤 후보도 §8의 측정 규율
(300시드 스크린 → 500+ 신선한 홀드아웃 결정, knob-only 컨트롤, 두 지표 동시 보고)을
벗어나서 채택될 수 없다.

---

## 1. 이 문제의 학술적 정체

우리 과제는 정확히 다음 세 문헌 분야의 교집합이다:

1. **탐색 결합 동적 차량 경로 문제** (DVR with search / limited sensing). 태스크가
   시간에 따라 확률적으로 생성되고, *발견해야만 서비스할 수 있으며*, 이동 예산이
   제한된다. Bullo–Frazzoli–Pavone의 서베이(Proc. IEEE 2011)가 이 계열의 지도이고,
   그 안의 limited-sensing 분기(Enright–Frazzoli)가 우리와 가장 가깝다.
2. **정보 수집 경로 계획** (informative path planning, IPP) = **예산 제약 오리엔티어링**.
   관측 가치를 최대화하는 경로를 이동 예산 안에서 짜는 문제. 관측 가치가 겹치므로
   submodular하고, 시간에 따라 변한다(스폰 램프). Correlated/time-varying-profit
   orienteering이 정확히 이 모양이다.
3. **예측적 디스패치** (anticipatory dispatching). 미래 수요를 분포로 알 때 현재
   결정을 어떻게 바꾸나 — MSA(시나리오 샘플링+합의), rollout(1-step 정책 개선),
   Ulmer의 전략적 대기.

우리가 이미 갖고 있는 것들을 이 좌표에 놓으면:

| 우리 것 | 문헌에서의 위치 | 평가 |
|---|---|---|
| 틱마다 정확한 fleet plan (subset DP + 분할) | TOP의 정확 해법 | **문헌 상단**. 경매(CBBA)류는 이것의 근사라 볼 것 없음 |
| hold-then-commit (t=800 크로스오버) | Ulmer의 "strategic waiting"과 동일 결론 | 문헌이 우리 측정을 독립적으로 재확인. 신규 아이디어 아님 |
| 관측 가치 = mass × serve, 경로 적분(path_gain) | IPP의 표준 VOI 모델 | 포인트별 가치 모델은 이미 문헌 수준 |
| 스카우트 = greedy argmax(가치/에너지) + 커밋 | IPP의 **가장 약한 부분**: greedy는 myopic | **여기가 문헌 대비 뒤처진 유일한 곳** → 후보 A/B |
| 워커 계획에 미래 스폰 미반영 | MSA/rollout이 다루는 지점 | §6의 실패들은 조잡한 버전이었음 → 후보 C |

## 2. "15개 발견 / 14개 완료 예시"에 대한 솔직한 평가

- 현재 빌드의 시드별 분산은 ±6 완료다. **seed 45에서 현재 빌드는 이미 14/14를 낸다**
  (`./bench 45 16` → discovered 14, completed 14). 한 번의 실행에서 15/14를 보는 것은
  현재 빌드로도 드물지 않다. `main.cpp`는 `srand(time(NULL))`이므로 남이 보여준
  "한 판"은 분포의 오른쪽 꼬리일 가능성이 가장 크다.
- **평균** 14 완료는 다른 얘기다: 검증된 전지적 정확 최적이 14.95–15.21이고, 현재
  스케줄러에 공짜 완전 정보를 줘도 14.23이다. 평균 14는 "정보 문제를 거의 완전히
  해결한 스케줄러"를 뜻하고, 불가능하진 않지만(상한 아래다) 남은 갭 전부를 요구한다.
- 결론: 그 예시는 목표 설정 근거로는 못 쓰지만, "정보 갭 ~2.3을 공격하라"는 이
  문서의 방향과는 일치한다. 아래 후보들은 전부 그 갭을 겨냥한다.

## 3. 분야별 조사 결과와 우리 문제에의 사상

### 3.1 탐색이론 (search theory) — 타이밍 미스의 이론

- **Koopman 이래의 고전 결과**: 탐지 수익이 노력에 대해 포화(오목)할 때, 최적 노력
  배분은 확률질량에 대해 **선형이 아니라 오목**하다.
- **"Optimum Search for Objects Having Unknown Arrival Times"** (Operations Research
  1959): 대상이 포아송으로 *도착*하고 탐지 지연에 따라 가치가 감소할 때의 최적
  재방문 스케줄 — **우리의 타이밍 미스(1.57/런) 문제가 1959년에 정의된 그 문제다**.
  질적 처방: 위치별 재관측 주기를 (도착률 × 가치감쇠)의 한계이득이 균형되게.
- **DTRP + 제한 시야** (Enright–Frazzoli; Bullo 서베이 [19]): 탐지 반경이 작아 수색이
  지배할 때, 최적 정책은 갱신율 λ인 영역을 **√λ에 비례하는 빈도**로 수색한다.
  선형 배분(우리의 greedy 가치/에너지)은 고질량 영역에 과투자한다는 함의.

### 3.2 시간가변 보상 오리엔티어링 — 스카우트 투어의 올바른 형식화

- **Correlated Orienteering** (Yu–Schwager–Rus, arXiv:1402.1896): 지속 감시 투어를
  이웃 상관 이차 효용의 오리엔티어링으로 풀며, MIQP로 **150노드까지 anytime**으로
  near-optimal. 우리 스카우트 문제를 창 중심 ~40–80 웨이포인트로 거칠게 만들면 그
  규모 안이다.
- **Team Orienteering with Time-Varying Profit** (INFORMS JoC 2021) 및 시간의존
  오리엔티어링 계열: 방문 시각에 따라 보상이 변하는 정확/근사 해법들. 우리 보상은
  방문 시각의 계단 함수(스폰 틱마다 증가)로 정확히 이 모양이다.
- 함의: "스폰 직후에 그 영역을 다시 봐라" 같은 위상 정렬은 손으로 설계하면 죽었지만
  (§6 two-phase pacing −0.18~−0.30), **시간 인덱스가 있는 투어 해법은 그것을 자동으로
  발견한다**. 실패한 것은 위상 정렬이라는 목표가 아니라 예산 재배분이라는 수단이었다.

### 3.3 예측적 디스패치 — 워커 쪽의 남은 채널

- **MSA** (Bent–Van Hentenryck 2004): 미래 도착을 샘플한 시나리오 K개를 각각 풀고
  현재 행동을 합의로 뽑는다. 동적 VRP의 표준 예측 기법.
- **Rollout** (Bertsekas; Secomandi): 후보 행동별로 기저 정책을 끝까지 굴려 평가하는
  1-step 정책 개선. 순차 일관성 조건에서 기저 정책보다 나빠지지 않는다.
- **Ulmer의 전략적 대기**: 늦은 수요가 많을수록 대기가 유리 — 우리의 hold-then-commit이
  정확히 이 결론의 측정된 버전. (신규 아이디어가 아니라 검증으로 기록.)
- 근거가 되는 우리 데이터: NOFORE "바운드"를 실제 런의 20%가 이긴다(§3a) — 발견이
  떨어질 때 워커가 *이미 그 방향으로 가고 있던* 경우다. 이 채널은 실재하고, MSA는
  정확히 이것(가는 김에 줍기)을 극대화하는 원리적 방법이다.

### 3.4 조사했고 **부적합 판정**한 분야 (이유와 함께)

| 분야 | 부적합 이유 |
|---|---|
| 에르고딕 커버리지 (Mathew–Mezić SMC 등) | 시간 평균 밀도를 목표 분포에 맞추는 프레임 — 우리는 수명이 ~79步인 *한 번의* 투어이고 벽 있는 격자라 스펙트럴 기반이 안 맞음. 후보 A가 같은 목표를 이산적으로 더 직접 달성 |
| 경매/CBBA 등 시장 기반 MRTA | 분산 환경용 근사 — 우리는 중앙집중 + **정확** 해법을 이미 가짐 |
| 풀 POMDP / MCTS(POMCP·DESPOT) | 상태공간(20×20 맵 posterior × 6로봇)에서 틱당 예산 내 불가능. rollout(후보 B)이 실현 가능한 절단판 |
| End-to-end RL / 심층 정책 | 제출물이 C++14 단일 파일, 맵이 시드마다 다름, 학습-배포 갭. 단, "오프라인 파라미터 최적화"라는 알맹이는 후보 E로 수용 |
| MAPF(경로 충돌 해소) | 로봇이 겹칠 수 있어(README §4.1) 충돌이 존재하지 않음 |

## 4. 실험 후보 랭킹

공통 전제: 어떤 후보도 +2.0을 주지 않는다. §3a의 상한 구조상 이들의 합리적 기대값은
각 +0.05~+0.3, 전부 합쳐 잘 되면 +0.5 안팎이다. 그걸 넘는 주장을 하는 후보가 있다면
그 측정이 틀린 것이다.

### A. 스카우트 투어의 전역 최적화 — 시간가변 보상 오리엔티어링 (1순위)

- **무엇을**: greedy argmax(가치/에너지)+커밋(schedular.cpp `path_gain`/스카우트 타겟
  선택부)을, 창 중심 웨이포인트 ~40–80개 위의 **시간 인덱스 투어 해법**으로 교체.
  보상 r(c, t_방문) = 그 시각까지 쌓인 미발견 질량(스폰 틱은 램프로 이미 알고 있음)
  × 발견 시각 기준 서비스 가능성. 예산 = 남은 에너지(페이싱 라인은 제약 또는 벌점).
  해법은 (웨이포인트, 거친 시간) 상태의 DP + 빔서치면 충분 — MIQP 없이 C++14 안에서 됨.
- **왜 새로움**: §6이 죽인 것은 (i) 시간 맹목적인 고정 기하 투어(격자/링/부스트로피던)와
  (ii) greedy의 국소 수선(recommit, band release)이다. **풀어낸 시간 인덱스 투어는
  시도된 적 없다.** 타이밍 미스가 커버리지 홀보다 큰 지금(1.57 vs 0.72), 시간 축을
  결정 변수로 갖는 유일한 후보다.
- **리스크**: path_gain 수정 이후 greedy가 이미 꽤 좋다. 드론 수명이 ~79步라 투어
  공간 자체가 작고, greedy가 근사적으로 최적일 수 있다. 그래서 먼저 B로 싸게 검증.
- **측정**: 스크린 전에 오프라인 상한 체크 — 현재 빌드의 실제 스카우트 궤적 100시드를
  로그로 뽑고, 같은 시드의 사후 최적 투어(전지적, 같은 예산)와 발견-기여를 비교.
  갭이 ~0.2 발견 미만이면 A/B 계열 전체를 접는다. **이 상한 체크가 첫 작업이다.**

### B. 스카우트 타겟 선택에 rollout — A의 저비용 파일럿 (2순위)

- **무엇을**: 타겟 후보 상위 K(~10)개 각각에 대해, "그 타겟을 먼저 간 뒤 현행 greedy
  정책으로 계속"을 가치 모델 위에서 시뮬레이션해 잔여 수명 누적 가치로 평가, argmax.
  기존 기계(가치 모델, 커밋, 페이싱) 전부 재사용. Bertsekas의 cost-improvement 성질이
  근거(대리 평가라 보장은 근사).
- **왜 먼저**: A와 같은 병(greedy myopia)을 겨냥하는 더 싼 약. **B가 ±0이면 A도 접는다**
  — 실험 순서 자체가 정보를 갖게 설계.
- **비용**: 후보 K × 잔여 스텝 시뮬레이션, 틱당이 아니라 커밋 경계에서만 실행하면
  런타임 영향 미미(현재 1.6s/시드).

### C. MSA 시나리오 합의 — 워커의 "가는 김에" 극대화 (3순위)

- **무엇을**: 커밋 이후(t≥800) 워커의 첫 leg 선택 시, 미래 스폰(틱은 알려짐, 위치는
  현재 미관측 질량에 비례해 샘플) 시나리오 K=8~16개를 만들어 **기존 정확 DP에 가상
  태스크로 추가해** 각각 풀고, 시나리오 간 합의(최다 선택 leg 또는 시나리오 평균
  완료수)로 첫 leg를 고른다. leg 경계에서만 풀면 계산 감당 가능.
- **왜 §6의 사망자들과 다른가**: "uniform 미래 스폰 대비 종점 가치 가중"(−0.18/−0.31),
  "예비 에너지 홀드백"(불채택)은 전부 *정적 휴리스틱*이었다. MSA는 예측을 **모든 것을
  가격 매기는 그 정확 플래너를 통해** 가격 매기므로, 시나리오들이 실제로 합의하는
  지점에서만 결정을 바꾼다. NOFORE 20% 이탈이 이 채널의 실존 증거.
- **리스크**: 인접 실패가 많다. 솔직한 사전 확률은 낮음(+0.05~0.1 기대). 다만 실패해도
  "예측 채널은 정확 플래너를 통해도 안 된다"는 종결 지식이 남는다.

### D. 제곱근 법칙 형태 실험 (4순위, 반나절짜리)

- **무엇을**: 스카우트 스코어의 질량 항을 영역 수준에서 오목화(mass^γ, γ≈0.5 스크린).
  스칼라 재스윕이 아니라 **함수 형태** 변경이고, 근거는 3.1의 √-법칙.
- **주의**: §6의 "never-seen 질량 ×2.5"(−0.11)는 반대 방향(볼록 부스트)이었다. 오목은
  미시도. 다만 SCOUT_K(locality offset)와 효과가 겹칠 수 있어 knob-only 컨트롤 필수.

### E. 조인트 파라미터 최적화 — algorithm configuration (5순위, 야간 배치)

- **무엇을**: 지금까지의 튜닝은 전부 **한 번에 한 놉**이었다. 살아있는 튜너블
  ~10개(SCOUT_K, DPACE, burst, SERVE_W_*, ENDGAME, PATROL_*)를 CMA-ES/irace식으로
  **동시에** 탐색해 상호작용을 찾는다. 문헌명: automated algorithm configuration.
- **비용**: 1.6s/시드 × 300시드 ÷ 4코어 ≈ 2분/평가. 하룻밤 ≈ 200~300 평가. 최종
  승자만 500+ 신선 홀드아웃 2개 풀로 확정 — §7의 "스크리닝 승자는 죽는다" 규율을
  루프 밖 최종 관문으로.
- **기대**: 낮은 창의성, 그러나 역사적으로 이런 데서 +0.05~0.15가 나옴. 실패해도
  "현 파라미터는 국소 최적"이라는 종결 지식.

### F. 기록만 하고 보류 — light-load DTRP 사전 배치

DTRP 경부하 정책(수요 분포의 1-median에 대기)은 워커 사전 배치를 시사하지만, exact
기준 사전 배치의 총가치가 0.24(15.21 vs 14.97)이고 §3b의 조기 이동 실패가 겹겹이다.
유일하게 남는 변형은 "커밋 후 첫 leg의 동점 해소를 스폰 질량 중심 쪽으로" 정도 —
C가 자동으로 포함하므로 별도 실험 불필요.

## 5. 제안 실행 순서 (다음 세션)

1. **A의 상한 체크** (사후 최적 스카우트 투어 vs 실제 궤적, 100시드) — 반나절.
   여기서 갭이 작으면 A/B 폐기, D/E로 직행. 이 순서가 세션 낭비를 막는다.
2. 갭이 있으면 **B(rollout)** 구현 → 300 스크린. B가 이기면 **A**로 확전.
3. **D**는 어느 경로든 끼워넣을 수 있는 반나절짜리.
4. **E**는 세션 종료 시 야간 배치로 걸어두는 용도.
5. **C**는 A/B 결론 이후 별도 세션 — 정확 DP에 가상 태스크를 넣는 배관 작업이 크다.

## 6. 출처

- Bullo, Frazzoli, Pavone, Savla, Smith, "Dynamic Vehicle Routing for Robotic Systems," Proc. IEEE, 2011. https://web.stanford.edu/~pavone/papers/Bullo.ea.IEEEProc10.pdf
- Yu, Schwager, Rus, "Correlated Orienteering Problem and its Application to Persistent Monitoring Tasks," 2014. https://arxiv.org/abs/1402.1896
- "Team Orienteering with Time-Varying Profit," INFORMS Journal on Computing, 2021. https://pubsonline.informs.org/doi/10.1287/ijoc.2020.1026
- Bent, Van Hentenryck, "Scenario-Based Planning for Partially Dynamic Vehicle Routing with Stochastic Customers," Operations Research, 2004. https://www.researchgate.net/publication/220244198
- Bertsekas, "Rollout Algorithms for Discrete Optimization: A Survey." https://web.mit.edu/dimitrib/www/Rollouts_Survey.pdf
- Posner, "Optimum Search for Objects Having Unknown Arrival Times," Operations Research 7(5), 1959. https://pubsonline.informs.org/doi/10.1287/opre.7.5.625
- Ulmer et al., 전략적 대기/예측 디스패치 계열: "The Same-Day Delivery Problem for Online Purchases," Transportation Science. https://pubsonline.informs.org/doi/10.1287/trsc.2016.0732
- Mathew, Mezić, "Spectral Multiscale Coverage," CDC 2009 (에르고딕 커버리지 — 부적합 판정 §3.4).
- 오리엔티어링 서베이 (2025): https://arxiv.org/abs/2512.16865
- Chekuri, Pál, recursive greedy for submodular orienteering; Hollinger, Sukhatme, sampling-based IPP (RIG). https://www.roboticsproceedings.org/rss09/p51.pdf
