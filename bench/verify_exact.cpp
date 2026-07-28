// Independent verification of bench/exact.cpp.
//
// exact.cpp claims to compute the true optimum of a relaxation of the
// simulator, and therefore a valid upper bound on any scheduler.  Two things
// have to hold for that claim, and this tool checks both, by methods that do
// not share code with the DP being checked:
//
//   A. ALGORITHM.  The subset DP over (S, last) with a Pareto frontier of
//      (energy, completion time) really does find every servable subset.
//      Checked against brute-force enumeration of *all permutations* of every
//      subset up to size K (default 6), evaluated by a direct simulation of
//      the route.  Also re-solves the partition step by an independent
//      recursive search and compares the optimum.
//
//   B. MODEL.  The cost model (Dijkstra step cost, work ticks, the tick at
//      which a robot becomes free again, the energy cap) matches what the
//      REAL simulator does.  Checked by replaying concrete routes over the
//      initial tasks in an actual MAP -- driving a real ROBOT through
//      start_moving/move/start_working/work exactly as main.cpp does -- and
//      comparing the predicted energy and completion tick against reality.
//      Only initial tasks are replayed, because a task dispatched later takes
//      its position from the RNG *and the current occupancy*, so moving a
//      robot changes where later tasks land and the two instances diverge.
//
// Usage: ./verify_exact <seed> [max_subset_size=6]
// Output: one line per check, plus "VERIFY <seed> OK" / "... FAIL".
#include "../simulator.h"
#include <cstdlib>
#include <queue>
#include <algorithm>

static const int INF = 1000000000;
static const int MAP_SIZE = 20, NUM_ROBOT = 6, NT = 16, WALL_DENSITY = 20;
static const int TIME_MAX = MAP_SIZE * 100;
static const int ROBOT_ENERGY = TIME_MAX * 6;

// Dijkstra that also records a parent, so a concrete path can be replayed.
static void dijkstra(const vector<vector<vector<int>>> &cost, int n, Coord src, int type,
                     vector<int> &d, vector<int> &par)
{
    d.assign(n * n, INF);
    par.assign(n * n, -1);
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
    int s = src.x * n + src.y;
    d[s] = 0;
    pq.push(make_pair(0, s));
    static const int DX[4] = {0, 0, -1, 1}, DY[4] = {1, -1, 0, 0};
    while (!pq.empty())
    {
        pair<int, int> t = pq.top();
        pq.pop();
        if (t.first != d[t.second])
            continue;
        int ux = t.second / n, uy = t.second % n;
        int cu = cost[ux][uy][type];
        if (cu == INFINITE || cu < 0)
            continue;
        for (int k = 0; k < 4; ++k)
        {
            int vx = ux + DX[k], vy = uy + DY[k];
            if (vx < 0 || vy < 0 || vx >= n || vy >= n)
                continue;
            int cv = cost[vx][vy][type];
            if (cv == INFINITE || cv < 0)
                continue;
            int w = ((cu / 2 + cv) + 9) / 10 * 10;
            if (t.first + w < d[vx * n + vy])
            {
                d[vx * n + vy] = t.first + w;
                par[vx * n + vy] = t.second;
                pq.push(make_pair(t.first + w, vx * n + vy));
            }
        }
    }
}

struct Inst
{
    vector<int> wtype;
    vector<Coord> wpos;
    vector<int> wid; // robot id in map.get_robots()
    int NW;
    int nt;
    vector<Coord> tpos;
    vector<int> release;
    vector<vector<int>> work;              // [task][type]
    vector<vector<int>> dstart;            // [worker][task]
    vector<vector<vector<int>>> dtt;       // [type][task][task]
};

// Build the instance exactly the way exact.cpp does.
static void build(unsigned seed, Inst &in, MAP **out_map)
{
    srand(seed);
    MAP *map = new MAP(MAP_SIZE, NUM_ROBOT, NT / 2, NT, WALL_DENSITY, ROBOT_ENERGY);
    TASKDISPATCHER disp(*map, TIME_MAX);
    for (int t = 0; t < TIME_MAX; ++t)
        disp.try_dispatch(t);
    set<Coord> all;
    for (int x = 0; x < MAP_SIZE; ++x)
        for (int y = 0; y < MAP_SIZE; ++y)
            all.emplace(x, y);
    map->update_coords(all);
    const vector<vector<vector<int>>> &cost = map->get_known_cost_map();

    int rid = 0;
    for (auto &r : map->get_robots())
    {
        if (r->type != ROBOT::TYPE::DRONE)
        {
            in.wtype.push_back(static_cast<int>(r->type));
            in.wpos.push_back(r->get_coord());
            in.wid.push_back(rid);
        }
        ++rid;
    }
    in.NW = static_cast<int>(in.wpos.size());

    auto &tasks = map->get_tasks();
    in.nt = static_cast<int>(tasks.size());
    in.release.assign(in.nt, 0);
    in.work.assign(in.nt, vector<int>(3, INF));
    for (int i = 0; i < in.nt; ++i)
    {
        in.tpos.push_back(tasks[i]->coord);
        in.release[i] = (i < NT / 2) ? 0 : TIME_MAX / 4 + (i - NT / 2) * TIME_MAX / NT;
        for (int ty = 1; ty <= 2; ++ty)
        {
            int c = tasks[i]->get_cost(static_cast<ROBOT::TYPE>(ty));
            int ticks = (c + 9) / 10;
            if (ticks < 1)
                ticks = 1;
            in.work[i][ty] = ticks * 10;
        }
    }
    vector<int> d, par;
    in.dstart.assign(in.NW, vector<int>(in.nt, INF));
    for (int w = 0; w < in.NW; ++w)
    {
        dijkstra(cost, MAP_SIZE, in.wpos[w], in.wtype[w], d, par);
        for (int j = 0; j < in.nt; ++j)
            in.dstart[w][j] = d[in.tpos[j].x * MAP_SIZE + in.tpos[j].y];
    }
    in.dtt.assign(3, vector<vector<int>>(in.nt, vector<int>(in.nt, INF)));
    for (int ty = 1; ty <= 2; ++ty)
        for (int i = 0; i < in.nt; ++i)
        {
            dijkstra(cost, MAP_SIZE, in.tpos[i], ty, d, par);
            for (int j = 0; j < in.nt; ++j)
                in.dtt[ty][i][j] = d[in.tpos[j].x * MAP_SIZE + in.tpos[j].y];
        }
    *out_map = map;
}

// ---- the DP under test (same algorithm as exact.cpp) ----------------------
static void dp_feasible(const Inst &in, bool nofore, vector<vector<char>> &feas)
{
    const int FULL = 1 << in.nt;
    feas.assign(in.NW, vector<char>(FULL, 0));
    for (int w = 0; w < in.NW; ++w)
    {
        int ty = in.wtype[w];
        vector<vector<pair<int, int>>> par(static_cast<size_t>(FULL) * in.nt);
        auto add = [&](size_t key, int e, int t) {
            vector<pair<int, int>> &v = par[key];
            for (size_t i = 0; i < v.size(); ++i)
                if (v[i].first <= e && v[i].second <= t)
                    return;
            for (size_t i = 0; i < v.size();)
            {
                if (v[i].first >= e && v[i].second >= t)
                    v.erase(v.begin() + i);
                else
                    ++i;
            }
            v.push_back(make_pair(e, t));
        };
        for (int j = 0; j < in.nt; ++j)
        {
            if (in.dstart[w][j] >= INF || in.work[j][ty] >= INF)
                continue;
            int e = in.dstart[w][j] + in.work[j][ty];
            if (e > ROBOT_ENERGY)
                continue;
            int depart = nofore ? in.release[j] : 0;
            int start = max(depart + in.dstart[w][j] / 10, in.release[j]);
            int t = start + in.work[j][ty] / 10;
            if (t > TIME_MAX)
                continue;
            add(static_cast<size_t>(1 << j) * in.nt + j, e, t);
            feas[w][1 << j] = 1;
        }
        vector<int> order;
        for (int S = 1; S < FULL; ++S)
            order.push_back(S);
        sort(order.begin(), order.end(), [](int a, int b) {
            return __builtin_popcount(a) < __builtin_popcount(b);
        });
        for (size_t oi = 0; oi < order.size(); ++oi)
        {
            int S = order[oi];
            for (int last = 0; last < in.nt; ++last)
            {
                if (!((S >> last) & 1))
                    continue;
                const vector<pair<int, int>> cur = par[static_cast<size_t>(S) * in.nt + last];
                if (cur.empty())
                    continue;
                feas[w][S] = 1;
                for (int j = 0; j < in.nt; ++j)
                {
                    if ((S >> j) & 1)
                        continue;
                    int leg = in.dtt[ty][last][j];
                    if (leg >= INF || in.work[j][ty] >= INF)
                        continue;
                    for (size_t c = 0; c < cur.size(); ++c)
                    {
                        int e = cur[c].first + leg + in.work[j][ty];
                        if (e > ROBOT_ENERGY)
                            continue;
                        int depart = cur[c].second;
                        if (nofore && depart < in.release[j])
                            depart = in.release[j];
                        int start = max(depart + leg / 10, in.release[j]);
                        int t = start + in.work[j][ty] / 10;
                        if (t > TIME_MAX)
                            continue;
                        add(static_cast<size_t>(S | (1 << j)) * in.nt + j, e, t);
                    }
                }
            }
        }
    }
}

// ---- BRUTE FORCE: evaluate one concrete route directly --------------------
// Returns true if worker w can serve the tasks in this exact order.
static bool route_ok(const Inst &in, int w, const vector<int> &route, bool nofore,
                     int *out_e = 0, int *out_t = 0)
{
    int ty = in.wtype[w];
    int e = 0, t = 0;
    for (size_t k = 0; k < route.size(); ++k)
    {
        int j = route[k];
        int leg = (k == 0) ? in.dstart[w][j] : in.dtt[ty][route[k - 1]][j];
        if (leg >= INF || in.work[j][ty] >= INF)
            return false;
        e += leg + in.work[j][ty];
        if (e > ROBOT_ENERGY)
            return false;
        int depart = (k == 0) ? (nofore ? in.release[j] : 0) : t;
        if (nofore && depart < in.release[j])
            depart = in.release[j];
        int start = max(depart + leg / 10, in.release[j]);
        t = start + in.work[j][ty] / 10;
        if (t > TIME_MAX)
            return false;
    }
    if (out_e) *out_e = e;
    if (out_t) *out_t = t;
    return true;
}

static bool brute_feasible(const Inst &in, int w, int S, bool nofore)
{
    vector<int> items;
    for (int j = 0; j < in.nt; ++j)
        if ((S >> j) & 1)
            items.push_back(j);
    sort(items.begin(), items.end());
    do
    {
        if (route_ok(in, w, items, nofore))
            return true;
    } while (next_permutation(items.begin(), items.end()));
    return false;
}

// ---- independent partition solver (recursive, not the layered DP) ---------
static bool part_ok(const Inst &in, const vector<vector<char>> &feas, int S, int w,
                    vector<vector<char>> &memo)
{
    if (w == in.NW)
        return S == 0;
    if (memo[w][S] != -1)
        return memo[w][S] != 0;
    memo[w][S] = 0;
    // worker w takes any subset T of S (including none)
    for (int T = S;; T = (T - 1) & S)
    {
        if (feas[w][T] || T == 0)
            if (part_ok(in, feas, S & ~T, w + 1, memo))
            {
                memo[w][S] = 1;
                break;
            }
        if (T == 0)
            break;
    }
    return memo[w][S] != 0;
}

int main(int argc, char **argv)
{
    unsigned seed = (argc > 1) ? static_cast<unsigned>(strtoul(argv[1], 0, 10)) : 1u;
    int K = (argc > 2) ? atoi(argv[2]) : 6;
    bool nofore = false;

    Inst in;
    MAP *map = 0;
    build(seed, in, &map);

    vector<vector<char>> feas;
    dp_feasible(in, nofore, feas);

    bool ok = true;
    const int FULL = 1 << in.nt;

    // ---- CHECK A1: DP feasibility vs brute-force permutations -------------
    long checked = 0, disagree = 0;
    for (int w = 0; w < in.NW; ++w)
        for (int S = 1; S < FULL; ++S)
        {
            if (__builtin_popcount(S) > K)
                continue;
            bool b = brute_feasible(in, w, S, nofore);
            bool a = feas[w][S] != 0;
            ++checked;
            if (a != b)
            {
                ++disagree;
                if (disagree <= 5)
                    cout << "  MISMATCH w=" << w << " S=" << S << " dp=" << a << " brute=" << b << endl;
            }
        }
    cout << "A1 feasibility: " << checked << " subsets (|S|<=" << K << ") checked, "
         << disagree << " mismatches" << endl;
    if (disagree)
        ok = false;

    // ---- CHECK A2: partition optimum, independent recursive solver --------
    vector<signed char> dp(FULL, -1), ndp(FULL, -1);
    dp[0] = 0;
    for (int w = 0; w < in.NW; ++w)
    {
        ndp = dp;
        for (int S = 0; S < FULL; ++S)
        {
            if (dp[S] < 0)
                continue;
            int rest = (FULL - 1) & ~S;
            for (int T = rest; T; T = (T - 1) & rest)
            {
                if (!feas[w][T])
                    continue;
                int v = dp[S] + __builtin_popcount(T);
                if (v > ndp[S | T])
                    ndp[S | T] = static_cast<signed char>(v);
            }
        }
        dp.swap(ndp);
    }
    int best_dp = 0;
    for (int S = 0; S < FULL; ++S)
        if (dp[S] > best_dp)
            best_dp = dp[S];

    int best_rec = 0;
    {
        vector<vector<char>> memo(in.NW + 1, vector<char>(FULL, -1));
        vector<int> by_pop;
        for (int S = 0; S < FULL; ++S)
            by_pop.push_back(S);
        sort(by_pop.begin(), by_pop.end(), [](int a, int b) {
            return __builtin_popcount(a) > __builtin_popcount(b);
        });
        for (size_t i = 0; i < by_pop.size(); ++i)
        {
            int S = by_pop[i];
            if (__builtin_popcount(S) <= best_rec)
                break;
            if (part_ok(in, feas, S, 0, memo))
            {
                best_rec = __builtin_popcount(S);
                break;
            }
        }
    }
    cout << "A2 partition: layered_dp=" << best_dp << " recursive=" << best_rec
         << (best_dp == best_rec ? "  match" : "  MISMATCH") << endl;
    if (best_dp != best_rec)
        ok = false;

    // ---- CHECK B: replay a concrete route in the REAL simulator -----------
    // Only initial tasks (release 0): a later dispatch would land somewhere
    // else once the robot has moved, so the instances would diverge.
    int replays = 0, replay_bad = 0;
    for (int w = 0; w < in.NW && replays < 3; ++w)
    {
        // greedy nearest-first route over up to 3 initial tasks
        vector<int> route;
        int cur = -1, ty = in.wtype[w];
        for (int step = 0; step < 3; ++step)
        {
            int bj = -1, bd = INF;
            for (int j = 0; j < NT / 2 && j < in.nt; ++j)
            {
                if (find(route.begin(), route.end(), j) != route.end())
                    continue;
                int leg = (cur < 0) ? in.dstart[w][j] : in.dtt[ty][cur][j];
                if (leg < bd)
                {
                    bd = leg;
                    bj = j;
                }
            }
            if (bj < 0)
                break;
            route.push_back(bj);
            cur = bj;
        }
        int pe = 0, pt = 0;
        if (route.empty() || !route_ok(in, w, route, false, &pe, &pt))
            continue;

        // --- now execute exactly that route in a fresh, real simulation ---
        srand(seed);
        MAP m2(MAP_SIZE, NUM_ROBOT, NT / 2, NT, WALL_DENSITY, ROBOT_ENERGY);
        TASKDISPATCHER d2(m2, TIME_MAX);
        auto &robots = m2.get_robots();
        auto &kom = m2.get_known_object_map();
        // reveal everything so the replay can path freely
        {
            set<Coord> all;
            for (int x = 0; x < MAP_SIZE; ++x)
                for (int y = 0; y < MAP_SIZE; ++y)
                    all.emplace(x, y);
            m2.update_coords(all);
        }
        const vector<vector<vector<int>>> &cm = m2.get_known_cost_map();
        auto robot = robots[in.wid[w]];
        size_t leg_i = 0;
        vector<int> path; // cells still to walk for the current leg
        int last_complete_tick = -1;
        int t = -1;
        while (++t < TIME_MAX && leg_i < route.size())
        {
            d2.try_dispatch(t);
            m2.update_coords(m2.observed_coord_by_robot());
            auto &status = robot->get_status();
            if (status == ROBOT::STATUS::IDLE)
            {
                Coord here = robot->get_coord();
                Coord goal = in.tpos[route[leg_i]];
                if (here == goal)
                {
                    if (bool(kom[here.x][here.y] & OBJECT::TASK))
                    {
                        robot->start_working(m2.task_at(here));
                    }
                    else // task already done -> advance
                    {
                        ++leg_i;
                        path.clear();
                        continue;
                    }
                }
                else
                {
                    if (path.empty())
                    {
                        vector<int> d, par;
                        dijkstra(cm, MAP_SIZE, here, in.wtype[w], d, par);
                        int node = goal.x * MAP_SIZE + goal.y;
                        while (node != here.x * MAP_SIZE + here.y && node >= 0)
                        {
                            path.push_back(node);
                            node = par[node];
                        }
                        reverse(path.begin(), path.end());
                    }
                    int nxt = path.front();
                    path.erase(path.begin());
                    int dx = nxt / MAP_SIZE - here.x, dy = nxt % MAP_SIZE - here.y;
                    ROBOT::ACTION a = (dy == 1)    ? ROBOT::ACTION::UP
                                      : (dy == -1) ? ROBOT::ACTION::DOWN
                                      : (dx == -1) ? ROBOT::ACTION::LEFT
                                                   : ROBOT::ACTION::RIGHT;
                    robot->start_moving(a);
                }
            }
            int before = m2.get_completed_task_num();
            if (status == ROBOT::STATUS::MOVING)
                robot->move();
            else if (status == ROBOT::STATUS::WORKING)
                robot->work();
            if (m2.get_completed_task_num() > before)
            {
                last_complete_tick = t;
                ++leg_i;
                path.clear();
            }
        }
        int actual_e = ROBOT_ENERGY - robot->get_energy();
        // model says: becomes free again at tick pt, i.e. finishes during pt-1
        ++replays;
        bool bad = (actual_e != pe) || (last_complete_tick != pt - 1) ||
                   (m2.get_completed_task_num() != (int)route.size());
        if (bad)
        {
            ++replay_bad;
            cout << "  REPLAY MISMATCH w=" << w << " predicted(e=" << pe << ",free_at=" << pt
                 << ") actual(e=" << actual_e << ",last_done=" << last_complete_tick
                 << ",done=" << m2.get_completed_task_num() << "/" << route.size() << ")" << endl;
        }
    }
    cout << "B  replay: " << replays << " routes executed in the real simulator, "
         << replay_bad << " mismatches" << endl;
    if (replay_bad)
        ok = false;

    cout << "VERIFY " << seed << " optimum=" << best_dp << " " << (ok ? "OK" : "FAIL") << endl;
    delete map;
    return ok ? 0 : 1;
}
