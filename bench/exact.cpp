// EXACT optimum for one instance -- not a heuristic.
//
// bench/plan.cpp is insertion + local search, so its answer is only a *lower*
// bound on the optimum and proves nothing about what is impossible.  This tool
// solves the same instance exactly, by subset DP, and therefore gives a real
// upper bound on what any scheduler could ever achieve.
//
// The model is a RELAXATION of the simulator, deliberately, so that the answer
// is a valid upper bound:
//   * every task's position and release time is known from t=0
//   * routing is optimal and costs nothing to plan
//   * not one unit of energy is spent on observation
//   * a worker may pre-position and wait (waiting is free in the simulator too)
//   * the one-tick gap the simulator inserts between arriving and starting work
//     is ignored, which only ever helps the bound
//
// Method:
//   1. For each worker w and each task subset S, decide whether w can serve
//      exactly S in some order within its energy and the horizon.  Done by DP
//      over (S, last visited) carrying the Pareto frontier of
//      (energy spent, completion time) -- the two are not interchangeable
//      because waiting for a release costs time but no energy.
//   2. Partition the 16 tasks across the 4 workers to maximise the count, by
//      subset-sum DP over all S and all T subset of S.
//
// Usage: ./exact <seed>
// Output: seed,exact_optimum,num_tasks,unreachable
#include "../../simulator.h"
#include <cstdlib>
#include <queue>
#include <algorithm>

static const int INF = 1000000000;

static vector<int> dijkstra(const vector<vector<vector<int>>> &cost, int n, Coord src, int type)
{
    vector<int> d(n * n, INF);
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
                pq.push(make_pair(t.first + w, vx * n + vy));
            }
        }
    }
    return d;
}

int main(int argc, char **argv)
{
    unsigned int seed = (argc > 1) ? static_cast<unsigned int>(strtoul(argv[1], 0, 10)) : 1u;
    // "nofore": a worker may not set off toward a task before that task exists.
    // Still generous (it knows the position the instant the task spawns) but it
    // removes clairvoyant pre-positioning.
    const bool nofore = (argc > 2) && string(argv[2]) == string("nofore");

    const int MAP_SIZE = 20, NUM_ROBOT = 6, NT = 16, WALL_DENSITY = 20;
    const int TIME_MAX = MAP_SIZE * 100;
    const int ROBOT_ENERGY = TIME_MAX * 6;

    srand(seed);
    MAP map(MAP_SIZE, NUM_ROBOT, NT / 2, NT, WALL_DENSITY, ROBOT_ENERGY);
    TASKDISPATCHER disp(map, TIME_MAX);
    for (int t = 0; t < TIME_MAX; ++t)
        disp.try_dispatch(t);
    {
        set<Coord> all;
        for (int x = 0; x < MAP_SIZE; ++x)
            for (int y = 0; y < MAP_SIZE; ++y)
                all.emplace(x, y);
        map.update_coords(all);
    }
    const vector<vector<vector<int>>> &cost = map.get_known_cost_map();

    vector<int> wtype;
    vector<Coord> wpos;
    for (auto &r : map.get_robots())
    {
        if (r->type == ROBOT::TYPE::DRONE)
            continue;
        wtype.push_back(static_cast<int>(r->type));
        wpos.push_back(r->get_coord());
    }
    const int NW = static_cast<int>(wpos.size());

    auto &tasks = map.get_tasks();
    const int nt = static_cast<int>(tasks.size());
    vector<Coord> tpos;
    vector<int> release(nt);
    vector<vector<int>> work(nt, vector<int>(3, INF));
    for (int i = 0; i < nt; ++i)
    {
        tpos.push_back(tasks[i]->coord);
        release[i] = (i < NT / 2) ? 0 : TIME_MAX / 4 + (i - NT / 2) * TIME_MAX / NT;
        for (int ty = 1; ty <= 2; ++ty)
        {
            int c = tasks[i]->get_cost(static_cast<ROBOT::TYPE>(ty));
            int ticks = (c + 9) / 10;
            if (ticks < 1)
                ticks = 1;
            work[i][ty] = ticks * 10;
        }
    }

    vector<vector<int>> dstart(NW, vector<int>(nt, INF));
    for (int w = 0; w < NW; ++w)
    {
        vector<int> d = dijkstra(cost, MAP_SIZE, wpos[w], wtype[w]);
        for (int j = 0; j < nt; ++j)
            dstart[w][j] = d[tpos[j].x * MAP_SIZE + tpos[j].y];
    }
    vector<vector<vector<int>>> dtt(3, vector<vector<int>>(nt, vector<int>(nt, INF)));
    for (int ty = 1; ty <= 2; ++ty)
        for (int i = 0; i < nt; ++i)
        {
            vector<int> d = dijkstra(cost, MAP_SIZE, tpos[i], ty);
            for (int j = 0; j < nt; ++j)
                dtt[ty][i][j] = d[tpos[j].x * MAP_SIZE + tpos[j].y];
        }

    int unreachable = 0;
    for (int j = 0; j < nt; ++j)
    {
        bool any = false;
        for (int w = 0; w < NW && !any; ++w)
            if (dstart[w][j] < INF)
                any = true;
        if (!any)
            ++unreachable;
    }

    const int FULL = 1 << nt;

    // ---- step 1: which subsets can each worker serve, in some order? --------
    // state (S, last) -> Pareto frontier of (energy spent, completion time)
    vector<vector<char>> feas(NW, vector<char>(FULL, 0));
    for (int w = 0; w < NW; ++w)
    {
        int ty = wtype[w];
        vector<vector<pair<int, int>>> par(static_cast<size_t>(FULL) * nt);
        vector<int> order;
        for (int S = 1; S < FULL; ++S)
            order.push_back(S);
        sort(order.begin(), order.end(), [](int a, int b) {
            return __builtin_popcount(a) < __builtin_popcount(b);
        });

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

        for (int j = 0; j < nt; ++j)
        {
            if (dstart[w][j] >= INF || work[j][ty] >= INF)
                continue;
            int e = dstart[w][j] + work[j][ty];
            if (e > ROBOT_ENERGY)
                continue;
            int depart = nofore ? release[j] : 0;
            int start = max(depart + dstart[w][j] / 10, release[j]);
            int t = start + work[j][ty] / 10;
            if (t > TIME_MAX)
                continue;
            add(static_cast<size_t>(1 << j) * nt + j, e, t);
            feas[w][1 << j] = 1;
        }

        for (size_t oi = 0; oi < order.size(); ++oi)
        {
            int S = order[oi];
            for (int last = 0; last < nt; ++last)
            {
                if (!((S >> last) & 1))
                    continue;
                const vector<pair<int, int>> cur = par[static_cast<size_t>(S) * nt + last];
                if (cur.empty())
                    continue;
                feas[w][S] = 1;
                for (int j = 0; j < nt; ++j)
                {
                    if ((S >> j) & 1)
                        continue;
                    int leg = dtt[ty][last][j];
                    if (leg >= INF || work[j][ty] >= INF)
                        continue;
                    for (size_t c = 0; c < cur.size(); ++c)
                    {
                        int e = cur[c].first + leg + work[j][ty];
                        if (e > ROBOT_ENERGY)
                            continue;
                        int depart = cur[c].second;
                        if (nofore && depart < release[j])
                            depart = release[j];
                        int start = max(depart + leg / 10, release[j]);
                        int t = start + work[j][ty] / 10;
                        if (t > TIME_MAX)
                            continue;
                        add(static_cast<size_t>(S | (1 << j)) * nt + j, e, t);
                    }
                }
            }
        }
    }

    // ---- step 2: partition the tasks across workers to maximise the count ---
    vector<signed char> dp(FULL, -1), ndp(FULL, -1);
    dp[0] = 0;
    for (int w = 0; w < NW; ++w)
    {
        ndp = dp;
        for (int S = 0; S < FULL; ++S)
        {
            if (dp[S] < 0)
                continue;
            int rest = (FULL - 1) & ~S;
            // enumerate every subset T of the still-unused tasks
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
    int best = 0;
    for (int S = 0; S < FULL; ++S)
        if (dp[S] > best)
            best = dp[S];

    cout << seed << "," << best << "," << nt << "," << unreachable << endl;
    return 0;
}
