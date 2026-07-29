// scoutbound -- how much is the scout's GREEDY target rule leaving on the table?
//
// This is the screening tool for EXPLORATION.md candidate A ("solve the scout
// tour as a time-indexed orienteering problem" instead of picking targets
// greedily by value-per-energy).  It is meant to kill that candidate cheaply if
// the headroom is not there, before anyone implements it.
//
// WHAT IT MEASURES
//
// The scout's job is to arrange, for each cell, a *last observation time* that
// is as late as possible, because a task spawning at time s on cell c is
// discovered iff c is observed at some time >= s.  So with the spawn schedule
// known (8 tasks at t=0, then one each at 500,625,...,1375) and spawn locations
// uniform over open cells, a whole observation record is worth
//
//     V  =  (1 / N_open) * sum_over_cells  g( lastobs(cell) )
//     g(L) = number of spawn events released at or before L   (0 if L < 0)
//
// in units of expected tasks discovered.  This is exactly the objective the
// online value model (mass = spawns released since last seen) approximates, and
// V computed on the real run's observation record reproduces the real
// discovered count -- which is the tool's built-in validity check.
//
// THE COMPARISON THAT MATTERS
//
//   V_actual   the real run (all robots, real partial map knowledge)
//   V_workers  the real run with the drones deleted -- what the workers alone see
//   V_greedy   drones re-planned by a GREEDY value-per-step rule, FULL map knowledge
//   V_opt      drones re-planned by simulated annealing over tours, FULL map knowledge
//
// V_opt - V_greedy is the number candidate A is worth: same information, same
// budget, same pacing structure, greedy target choice vs optimised tour.
// V_greedy - V_actual is the separate gain from knowing the map in advance,
// which candidate A cannot capture and which is NOT evidence for it.
//
// DIRECTION OF THE ERRORS (so the answer is read correctly)
//   * the optimiser gets the true map for free; the online scout does not, so
//     V_opt overstates what any online method could reach.
//   * SA returns a solution, hence a LOWER bound on the true optimal tour.
//   * worker observation is held exactly as it happened.
// Therefore a SMALL V_opt - V_greedy is conclusive (an online method with less
// information and less compute cannot beat what full-information SA could not
// find), while a LARGE gap is only suggestive and would need the real thing.
//
// Usage:  ./scoutbound <seed> [iters=60000]
// Output: SCOUT,seed,disc_actual,V_actual,V_workers,V_greedy,V_opt,V_opt_fixpace,
//                drone_steps,step_ticks,n_open
#include "../../simulator.h"
#include "../../schedular.h"
#include <cstdlib>
#include <climits>
#include <queue>
#include <algorithm>
#include <cmath>

static const int TMAX = 2000;
static const int MSZ = 20;
static const int NC = MSZ * MSZ;
static const int DRONE_VIEW = 2;

// ---------------------------------------------------------------- rng
struct Rng
{
    unsigned long long s;
    Rng(unsigned long long seed) : s(seed ? seed : 0x9E3779B97F4A7C15ULL) {}
    unsigned int next()
    {
        s ^= s << 13;
        s ^= s >> 7;
        s ^= s << 17;
        return static_cast<unsigned int>(s >> 32);
    }
    int below(int n) { return static_cast<int>(next() % static_cast<unsigned int>(n)); }
    double unit() { return static_cast<double>(next()) / 4294967296.0; }
};

// ---------------------------------------------------------------- problem
struct Prob
{
    vector<char> open;              // non-wall
    int n_open;
    int sc;                         // ticks (and energy/10) per drone step
    vector<int> gtab;               // gtab[t] = spawn events released by tick t
    vector<vector<int>> win;        // win[c] = cells inside a drone's 5x5 at c
    vector<vector<int>> bd, bp;     // BFS steps / parent, per source cell
    vector<int> base_L;             // last-seen from the WORKERS only
    int start[2];                   // drone start cells
    int maxstep[2];                 // steps affordable
    int energy[2];

    int gof(int L) const { return (L < 0) ? 0 : gtab[L > TMAX ? TMAX : L]; }

    // exact value of a pair of drone walks, on top of the fixed worker record
    double eval(const vector<int> &w0, const vector<int> &w1,
                int pace0, int pace1, vector<int> &L) const
    {
        L = base_L;
        for (int d = 0; d < 2; ++d)
        {
            const vector<int> &w = d ? w1 : w0;
            if (w.empty())
                continue;
            int nsteps = static_cast<int>(w.size()) - 1;
            int pace = d ? pace1 : pace0;
            int iv = sc;
            if (nsteps > 0 && pace / nsteps > sc)
                iv = pace / nsteps;
            bool alive = (energy[d] - nsteps * 10 * sc) > 0;
            for (int i = 0; i < static_cast<int>(w.size()); ++i)
            {
                int arrive = i * iv;
                if (arrive > TMAX)
                    break;
                int depart;
                if (i + 1 < static_cast<int>(w.size()))
                    depart = (i + 1) * iv;
                else
                    depart = alive ? TMAX : arrive;
                if (depart > TMAX)
                    depart = TMAX;
                const vector<int> &wc = win[w[i]];
                for (size_t k = 0; k < wc.size(); ++k)
                    if (L[wc[k]] < depart)
                        L[wc[k]] = depart;
            }
        }
        long long tot = 0;
        for (int c = 0; c < NC; ++c)
            if (open[c])
                tot += gof(L[c]);
        return static_cast<double>(tot) / static_cast<double>(n_open);
    }

    // walk = start -> shortest path through each waypoint, capped at maxstep
    void build_walk(int d, const vector<int> &wps, vector<int> &out) const
    {
        out.clear();
        int cur = start[d];
        out.push_back(cur);
        int budget = maxstep[d];
        vector<int> seg;
        for (size_t j = 0; j < wps.size(); ++j)
        {
            int t = wps[j];
            if (t == cur || !open[t])
                continue;
            if (bd[cur][t] >= INT_MAX / 4)
                continue;
            seg.clear();
            for (int v = t; v != cur; v = bp[cur][v])
            {
                seg.push_back(v);
                if (bp[cur][v] < 0)
                    break;
            }
            reverse(seg.begin(), seg.end());
            for (size_t k = 0; k < seg.size(); ++k)
            {
                if (static_cast<int>(out.size()) - 1 >= budget)
                    return;
                out.push_back(seg[k]);
            }
            cur = t;
        }
    }
};

// ---------------------------------------------------------------- greedy
// A faithful analogue of the shipped scout rule: walk to the target maximising
// (value the whole trip brings in) / (steps + locality offset), commit to it,
// repeat.  The resulting walk is then scored by the SAME exact evaluator used
// for the optimiser, so no modelling asymmetry can favour one over the other.
static void greedy_tour(const Prob &P, int d, int pace, const vector<int> &Lstart,
                        vector<int> &wps, int LOCAL_K, int xlo, int xhi)
{
    wps.clear();
    vector<int> L = Lstart;
    int cur = P.start[d];
    int used = 0;
    int nsteps_est = P.maxstep[d];
    int iv = P.sc;
    if (nsteps_est > 0 && pace / nsteps_est > P.sc)
        iv = pace / nsteps_est;

    // credit the window the drone is standing in at t=0
    {
        const vector<int> &wc = P.win[cur];
        for (size_t k = 0; k < wc.size(); ++k)
            if (L[wc[k]] < 0)
                L[wc[k]] = 0;
    }

    while (used < P.maxstep[d])
    {
        int best = -1;
        double best_ratio = 0.0;
        const vector<int> &bdc = P.bd[cur];
        const vector<int> &bpc = P.bp[cur];

        // accumulate marginal gain over the shortest-path tree, parents first
        static vector<pair<int, int>> ord;
        ord.clear();
        for (int c = 0; c < NC; ++c)
            if (P.open[c] && bdc[c] < INT_MAX / 4)
                ord.push_back(make_pair(bdc[c], c));
        sort(ord.begin(), ord.end());
        static vector<double> pg;
        pg.assign(NC, 0.0);
        for (size_t i = 0; i < ord.size(); ++i)
        {
            int v = ord[i].second, u = bpc[v];
            if (u < 0)
                continue;
            int steps_here = bdc[v];
            if (used + steps_here > P.maxstep[d])
                continue;
            int at = (used + steps_here) * iv;
            if (at > TMAX)
                continue;
            // cells that enter view stepping u -> v
            double add = 0.0;
            int vx = v / MSZ, vy = v % MSZ, ux = u / MSZ, uy = u % MSZ;
            for (int xx = max(vx - DRONE_VIEW, 0); xx <= min(vx + DRONE_VIEW, MSZ - 1); ++xx)
                for (int yy = max(vy - DRONE_VIEW, 0); yy <= min(vy + DRONE_VIEW, MSZ - 1); ++yy)
                {
                    if (abs(xx - ux) <= DRONE_VIEW && abs(yy - uy) <= DRONE_VIEW)
                        continue; // already visible from u
                    int c = xx * MSZ + yy;
                    if (!P.open[c])
                        continue;
                    double g = P.gof(at) - P.gof(L[c]);
                    if (g > 0)
                        add += g;
                }
            pg[v] = pg[u] + add;
            int vxb = v / MSZ;
            if (vxb < xlo || vxb > xhi)
                continue; // outside this scout's band: not a candidate target
            double ratio = pg[v] / static_cast<double>(steps_here + LOCAL_K);
            if (ratio > best_ratio)
            {
                best_ratio = ratio;
                best = v;
            }
        }
        if (best < 0 || best_ratio <= 0.0)
            break;

        // commit: walk the path, updating the record as the drone goes
        vector<int> seg;
        for (int v = best; v != cur; v = bpc[v])
        {
            seg.push_back(v);
            if (bpc[v] < 0)
                break;
        }
        reverse(seg.begin(), seg.end());
        for (size_t k = 0; k < seg.size(); ++k)
        {
            if (used >= P.maxstep[d])
                break;
            ++used;
            int at = used * iv;
            if (at > TMAX)
                break;
            const vector<int> &wc = P.win[seg[k]];
            for (size_t q = 0; q < wc.size(); ++q)
                if (L[wc[q]] < at)
                    L[wc[q]] = at;
            cur = seg[k];
        }
        wps.push_back(cur);
        if (cur != best)
            break; // ran out mid-path
    }
}

int main(int argc, char **argv)
{
    unsigned int seed = (argc > 1) ? static_cast<unsigned int>(strtoul(argv[1], nullptr, 10)) : 0u;
    const int ITERS = (argc > 2) ? atoi(argv[2]) : 60000;
    const int NUM_MAX_TASKS = 16;
    const int NUM_INITIAL_TASKS = NUM_MAX_TASKS / 2;
    const int WALL_DENSITY = 20;
    const int ROBOT_ENERGY = TMAX * 6;

    srand(seed);
    MAP map(MSZ, 6, NUM_INITIAL_TASKS, NUM_MAX_TASKS, WALL_DENSITY, ROBOT_ENERGY);
    int time = -1;
    auto &robots = map.get_robots();
    auto &known_cost_map = map.get_known_cost_map();
    auto &known_object_map = map.get_known_object_map();
    auto &active_tasks = map.get_active_tasks();
    Scheduler scheduler;
    TASKDISPATCHER taskdispatcher(map, TMAX);
    set<Coord> observed_coords, updated_coords;

    // per-tick observation record, split drone / worker
    vector<int> L_all(NC, -1), L_work(NC, -1);
    int drone_start[2] = {-1, -1}, drone_energy0[2] = {0, 0};
    int drone_steps_actual = 0;
    {
        int k = 0;
        for (auto &r : robots)
            if (r->type == ROBOT::TYPE::DRONE && k < 2)
            {
                drone_start[k] = r->get_coord().x * MSZ + r->get_coord().y;
                drone_energy0[k] = r->get_energy();
                ++k;
            }
    }
    vector<Coord> prev_drone_pos(2, Coord(-1, -1));

    while (++time < TMAX &&
           robots.size() != map.get_exhausted_robot_num() &&
           map.num_total_task != map.get_completed_task_num())
    {
        taskdispatcher.try_dispatch(time);
        observed_coords = map.observed_coord_by_robot();

        // record the same windows observed_coord_by_robot() builds, but keeping
        // track of which robot class produced each one
        {
            int di = 0;
            for (auto &r : robots)
            {
                if (r->get_status() == ROBOT::STATUS::EXHAUSTED)
                {
                    if (r->type == ROBOT::TYPE::DRONE)
                        ++di;
                    continue;
                }
                int vr = ROBOT::view_range_list[static_cast<size_t>(r->type)];
                bool cross = (ROBOT::view_type_list[static_cast<size_t>(r->type)] ==
                              ROBOT::VIEWTYPE::CROSS);
                int x = r->get_coord().x, y = r->get_coord().y;
                bool is_drone = (r->type == ROBOT::TYPE::DRONE);
                for (int xx = max(x - vr, 0); xx <= min(x + vr, MSZ - 1); ++xx)
                    for (int yy = max(y - vr, 0); yy <= min(y + vr, MSZ - 1); ++yy)
                    {
                        if (cross && xx != x && yy != y)
                            continue;
                        int c = xx * MSZ + yy;
                        if (L_all[c] < time)
                            L_all[c] = time;
                        if (!is_drone && L_work[c] < time)
                            L_work[c] = time;
                    }
                if (is_drone && di < 2)
                {
                    if (!(prev_drone_pos[di] == r->get_coord()) && prev_drone_pos[di].x >= 0)
                        ++drone_steps_actual;
                    prev_drone_pos[di] = r->get_coord();
                    ++di;
                }
            }
        }

        updated_coords = map.update_coords(observed_coords);
        scheduler.on_info_updated(observed_coords, updated_coords, known_cost_map,
                                  known_object_map, active_tasks, robots);
        for (auto robot : robots)
        {
            auto &status = robot->get_status();
            if (status == ROBOT::STATUS::IDLE)
            {
                auto coord = robot->get_coord();
                bool do_task = false;
                weak_ptr<TASK> task;
                if (bool(known_object_map[coord.x][coord.y] & OBJECT::TASK))
                {
                    task = map.task_at(coord);
                    do_task = scheduler.on_task_reached(observed_coords, updated_coords,
                                                        known_cost_map, known_object_map,
                                                        active_tasks, robots, *robot,
                                                        *(task.lock()));
                }
                if (do_task)
                    robot->start_working(task);
                else
                    robot->start_moving(scheduler.idle_action(observed_coords, updated_coords,
                                                              known_cost_map, known_object_map,
                                                              active_tasks, robots, *robot));
            }
            if (status == ROBOT::STATUS::MOVING)
                robot->move();
            else if (status == ROBOT::STATUS::WORKING)
                robot->work();
        }
    }

    int completed = map.get_completed_task_num();
    int discovered = completed + static_cast<int>(active_tasks.size());

    // ---- reveal the true map and set the problem up ----------------------
    {
        set<Coord> all;
        for (int x = 0; x < MSZ; ++x)
            for (int y = 0; y < MSZ; ++y)
                all.emplace(x, y);
        map.update_coords(all);
    }
    Prob P;
    P.open.assign(NC, 0);
    P.n_open = 0;
    for (int x = 0; x < MSZ; ++x)
        for (int y = 0; y < MSZ; ++y)
            if (map.get_cost({x, y}, ROBOT::TYPE::CATERPILLAR) != INFINITE)
            {
                P.open[x * MSZ + y] = 1;
                ++P.n_open;
            }
    int dcost = -1;
    for (int x = 0; x < MSZ && dcost < 0; ++x)
        for (int y = 0; y < MSZ; ++y)
        {
            int c = map.get_cost({x, y}, ROBOT::TYPE::DRONE);
            if (c > 0 && c != INFINITE)
            {
                dcost = c;
                break;
            }
        }
    P.sc = (dcost / 2 + dcost + 9) / 10; // ticks per drone step
    if (P.sc < 1)
        P.sc = 1;

    P.gtab.assign(TMAX + 1, 0);
    {
        vector<int> spawns;
        for (int i = 0; i < NUM_INITIAL_TASKS; ++i)
            spawns.push_back(0);
        int t0 = TMAX / 4, inc = TMAX / NUM_MAX_TASKS;
        for (int i = 0; i < NUM_MAX_TASKS - NUM_INITIAL_TASKS; ++i)
            spawns.push_back(t0 + inc * i);
        for (int t = 0; t <= TMAX; ++t)
        {
            int c = 0;
            for (size_t i = 0; i < spawns.size(); ++i)
                if (spawns[i] <= t)
                    ++c;
            P.gtab[t] = c;
        }
    }

    P.win.assign(NC, vector<int>());
    for (int x = 0; x < MSZ; ++x)
        for (int y = 0; y < MSZ; ++y)
        {
            int c = x * MSZ + y;
            if (!P.open[c])
                continue;
            for (int xx = max(x - DRONE_VIEW, 0); xx <= min(x + DRONE_VIEW, MSZ - 1); ++xx)
                for (int yy = max(y - DRONE_VIEW, 0); yy <= min(y + DRONE_VIEW, MSZ - 1); ++yy)
                    if (P.open[xx * MSZ + yy])
                        P.win[c].push_back(xx * MSZ + yy);
        }

    P.bd.assign(NC, vector<int>());
    P.bp.assign(NC, vector<int>());
    {
        static const int DX[4] = {0, 0, -1, 1}, DY[4] = {1, -1, 0, 0};
        for (int s = 0; s < NC; ++s)
        {
            if (!P.open[s])
                continue;
            P.bd[s].assign(NC, INT_MAX / 4);
            P.bp[s].assign(NC, -1);
            vector<int> q;
            q.push_back(s);
            P.bd[s][s] = 0;
            for (size_t h = 0; h < q.size(); ++h)
            {
                int u = q[h], ux = u / MSZ, uy = u % MSZ;
                for (int k = 0; k < 4; ++k)
                {
                    int vx = ux + DX[k], vy = uy + DY[k];
                    if (vx < 0 || vy < 0 || vx >= MSZ || vy >= MSZ)
                        continue;
                    int v = vx * MSZ + vy;
                    if (!P.open[v] || P.bd[s][v] < INT_MAX / 4)
                        continue;
                    P.bd[s][v] = P.bd[s][u] + 1;
                    P.bp[s][v] = u;
                    q.push_back(v);
                }
            }
        }
    }

    P.base_L = L_work;
    for (int d = 0; d < 2; ++d)
    {
        P.start[d] = drone_start[d];
        P.energy[d] = drone_energy0[d];
        int by_energy = drone_energy0[d] / (10 * P.sc);
        int by_time = TMAX / P.sc;
        P.maxstep[d] = min(by_energy, by_time);
    }

    vector<int> Ltmp;
    double V_actual = 0.0, V_workers = 0.0;
    {
        long long a = 0, w = 0;
        for (int c = 0; c < NC; ++c)
            if (P.open[c])
            {
                a += P.gof(L_all[c]);
                w += P.gof(L_work[c]);
            }
        V_actual = static_cast<double>(a) / P.n_open;
        V_workers = static_cast<double>(w) / P.n_open;
    }

    // ---- greedy, full map knowledge --------------------------------------
    // The baseline must not be a strawman, or the headroom this tool reports is
    // just the weakness of the baseline.  So build the strongest greedy
    // available: sweep the locality offset, and try both the unbanded rule and
    // the shipped x-band split (which is what stops the two scouts duplicating
    // each other's coverage), then keep the best.
    const int PACE0 = 1900; // the shipped DRONE_PACE_T
    vector<int> gw[2], gwalk[2];
    double V_greedy = -1.0;
    int best_k = -1, best_band = -1;
    {
        static const int KS[5] = {1, 2, 3, 5, 8};
        // bands assigned by starting x, exactly as the scheduler does
        int lo_drone = (P.start[0] >= 0 && P.start[1] >= 0 &&
                        (P.start[1] / MSZ) < (P.start[0] / MSZ)) ? 1 : 0;
        for (int band = 0; band < 2; ++band)
            for (int ki = 0; ki < 5; ++ki)
            {
                vector<int> cw[2], cwalk[2];
                vector<int> L = P.base_L;
                for (int d = 0; d < 2; ++d)
                {
                    if (P.start[d] < 0 || !P.open[P.start[d]])
                        continue;
                    int xlo = 0, xhi = MSZ - 1;
                    if (band)
                    {
                        bool first = (d == lo_drone);
                        xlo = first ? 0 : MSZ / 2;
                        xhi = first ? MSZ / 2 - 1 : MSZ - 1;
                    }
                    greedy_tour(P, d, PACE0, L, cw[d], KS[ki], xlo, xhi);
                    P.build_walk(d, cw[d], cwalk[d]);
                    int nsteps = static_cast<int>(cwalk[d].size()) - 1;
                    int iv = P.sc;
                    if (nsteps > 0 && PACE0 / nsteps > P.sc)
                        iv = PACE0 / nsteps;
                    bool alive = (P.energy[d] - nsteps * 10 * P.sc) > 0;
                    for (int i = 0; i < static_cast<int>(cwalk[d].size()); ++i)
                    {
                        int arrive = i * iv;
                        if (arrive > TMAX)
                            break;
                        int dep = (i + 1 < static_cast<int>(cwalk[d].size()))
                                      ? (i + 1) * iv
                                      : (alive ? TMAX : arrive);
                        if (dep > TMAX)
                            dep = TMAX;
                        for (size_t k = 0; k < P.win[cwalk[d][i]].size(); ++k)
                            if (L[P.win[cwalk[d][i]][k]] < dep)
                                L[P.win[cwalk[d][i]][k]] = dep;
                    }
                }
                double v = P.eval(cwalk[0], cwalk[1], PACE0, PACE0, Ltmp);
                if (v > V_greedy)
                {
                    V_greedy = v;
                    gw[0] = cw[0];
                    gw[1] = cw[1];
                    gwalk[0] = cwalk[0];
                    gwalk[1] = cwalk[1];
                    best_k = KS[ki];
                    best_band = band;
                }
            }
    }

    // ---- simulated annealing over tours ----------------------------------
    // Seeded from the greedy solution, so V_opt >= V_greedy by construction and
    // the difference is exactly what tour optimisation adds.
    vector<int> opens;
    for (int c = 0; c < NC; ++c)
        if (P.open[c])
            opens.push_back(c);

    Rng rng(seed * 2654435761u + 12345u);
    vector<int> sa_wp[2]; // best waypoints found, for the diagnostics below
    auto run_sa = [&](bool free_pace, double &out_val, int &out_p0, int &out_p1) {
        vector<int> wp[2];
        wp[0] = gw[0];
        wp[1] = gw[1];
        int pace[2] = {PACE0, PACE0};
        vector<int> wk0, wk1;
        P.build_walk(0, wp[0], wk0);
        P.build_walk(1, wp[1], wk1);
        double cur = P.eval(wk0, wk1, pace[0], pace[1], Ltmp);
        vector<int> bwp[2] = {wp[0], wp[1]};
        int bpace[2] = {pace[0], pace[1]};
        double best = cur;
        double T0 = 0.05, T1 = 0.0008;
        for (int it = 0; it < ITERS; ++it)
        {
            double T = T0 * pow(T1 / T0, static_cast<double>(it) / ITERS);
            int d = rng.below(2);
            if (P.start[d] < 0)
                d = 1 - d;
            if (P.start[d] < 0)
                break;
            vector<int> save = wp[d];
            int savep = pace[d];
            int op = rng.below(free_pace ? 6 : 5);
            if (op == 0 && !wp[d].empty()) // replace
                wp[d][rng.below(static_cast<int>(wp[d].size()))] = opens[rng.below((int)opens.size())];
            else if (op == 1) // insert
            {
                int at = wp[d].empty() ? 0 : rng.below(static_cast<int>(wp[d].size()) + 1);
                wp[d].insert(wp[d].begin() + at, opens[rng.below((int)opens.size())]);
            }
            else if (op == 2 && !wp[d].empty()) // delete
                wp[d].erase(wp[d].begin() + rng.below(static_cast<int>(wp[d].size())));
            else if (op == 3 && wp[d].size() >= 2) // swap
            {
                int a = rng.below(static_cast<int>(wp[d].size()));
                int b = rng.below(static_cast<int>(wp[d].size()));
                swap(wp[d][a], wp[d][b]);
            }
            else if (op == 4 && wp[d].size() >= 2) // reverse a segment
            {
                int a = rng.below(static_cast<int>(wp[d].size()));
                int b = rng.below(static_cast<int>(wp[d].size()));
                if (a > b)
                    swap(a, b);
                reverse(wp[d].begin() + a, wp[d].begin() + b + 1);
            }
            else if (op == 5) // re-pace
            {
                int np = pace[d] + (rng.below(2) ? 1 : -1) * (100 + rng.below(400));
                if (np < 0)
                    np = 0;
                if (np > TMAX)
                    np = TMAX;
                pace[d] = np;
            }
            P.build_walk(0, wp[0], wk0);
            P.build_walk(1, wp[1], wk1);
            double v = P.eval(wk0, wk1, pace[0], pace[1], Ltmp);
            if (v >= cur || rng.unit() < exp((v - cur) / T))
            {
                cur = v;
                if (v > best)
                {
                    best = v;
                    bwp[0] = wp[0];
                    bwp[1] = wp[1];
                    bpace[0] = pace[0];
                    bpace[1] = pace[1];
                }
            }
            else
            {
                wp[d] = save;
                pace[d] = savep;
            }
        }
        out_val = best;
        out_p0 = bpace[0];
        out_p1 = bpace[1];
        sa_wp[0] = bwp[0];
        sa_wp[1] = bwp[1];
    };

    double V_opt = 0.0, V_opt_fix = 0.0;
    int p0 = PACE0, p1 = PACE0, q0 = PACE0, q1 = PACE0;
    run_sa(true, V_opt, p0, p1);
    run_sa(false, V_opt_fix, q0, q1);

    // ---- diagnostics: WHAT does the optimised tour do differently? --------
    // Three shape statistics on the open cells, for the greedy tours and the
    // optimised ones, so the gap can be attributed rather than just reported.
    int unseen_g = 0, unseen_o = 0, late_g = 0, late_o = 0;
    long long sum_g = 0, sum_o = 0;
    {
        vector<int> Lg, Lo;
        P.eval(gwalk[0], gwalk[1], PACE0, PACE0, Lg);
        vector<int> ow0, ow1;
        P.build_walk(0, sa_wp[0], ow0);
        P.build_walk(1, sa_wp[1], ow1);
        P.eval(ow0, ow1, PACE0, PACE0, Lo);
        int last_spawn = TMAX / 4 + (TMAX / NUM_MAX_TASKS) * (NUM_MAX_TASKS - NUM_INITIAL_TASKS - 1);
        for (int c = 0; c < NC; ++c)
            if (P.open[c])
            {
                if (Lg[c] < 0)
                    ++unseen_g;
                else
                    sum_g += Lg[c];
                if (Lo[c] < 0)
                    ++unseen_o;
                else
                    sum_o += Lo[c];
                if (Lg[c] >= last_spawn)
                    ++late_g;
                if (Lo[c] >= last_spawn)
                    ++late_o;
            }
    }

    printf("SCOUT,%u,%d,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%lld,%lld\n",
           seed, discovered, completed, V_actual, V_workers, V_greedy, V_opt, V_opt_fix,
           drone_steps_actual, P.sc, P.n_open, p0, p1,
           best_k, best_band, unseen_g, unseen_o, late_g, late_o, sum_g, sum_o);
    return 0;
}
