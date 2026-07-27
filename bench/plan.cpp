// Offline upper-bound planner: how many of the 16 tasks could be completed by a
// PERFECT scheduler that knows the whole map and every task's position and
// release time from t=0?
//
// This is not a scheduler and is not part of the deliverable.  It exists to
// separate "our router is not good enough" from "the fleet physically cannot".
//
// Instance construction is scheduler-independent: the map is built from the
// seed, the robots never move, and the dispatcher is stepped through the whole
// run, so the 16 task positions depend only on the seed.  Costs are the true
// ones (revealed by observing every cell once, which is what the known cost map
// is for).
//
// Model (matches the simulator exactly):
//   * step u->v costs ceil((floor(cost(u)/2) + cost(v)) / 10) ticks, 10 energy each
//   * work costs 10 * max(1, ceil(task_cost/10)) energy, same in ticks
//   * holding is free in both currencies, so a worker may arrive early and wait
//   * a worker has ROBOT_ENERGY total; everything must finish before TIME_MAX
//
// Usage: ./plan <seed> [restarts=300] [notime]
// Output: seed,best_completed,greedy_completed,energy_used
#include "../../simulator.h"
#include <cstdlib>
#include <climits>
#include <queue>
#include <algorithm>

static const int INF = 1000000000;

int MAPN = 20;

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

struct Inst
{
    int nw;                       // workers
    vector<int> wtype;            // per worker: 1 cat, 2 wheel
    vector<int> energy;           // per worker budget
    int horizon;
    bool nofore;
    int nt;                       // tasks
    vector<int> release;          // per task
    vector<vector<int>> work;     // [task][type] energy
    vector<vector<int>> dstart;   // [worker][task] travel energy
    vector<vector<vector<int>>> dtt; // [type][task][task]
};

// Evaluate one worker's ordered task list: returns how many of the prefix fit
// within energy and horizon (the sequence is truncated at the first violation).
static int eval_seq(const Inst &in, int w, const vector<int> &seq, int *energy_out)
{
    int e = 0, t = 0, done = 0, prev = -1;
    int ty = in.wtype[w];
    for (size_t i = 0; i < seq.size(); ++i)
    {
        int j = seq[i];
        int trav = (prev < 0) ? in.dstart[w][j] : in.dtt[ty][prev][j];
        if (trav >= INF)
            break;
        int wk = in.work[j][ty];
        if (e + trav + wk > in.energy[w])
            break;
        int depart = t;
        if (in.nofore && depart < in.release[j])
            depart = in.release[j]; // no clairvoyance: cannot set off before it exists
        int arrive = depart + trav / 10;
        if (arrive < in.release[j])
            arrive = in.release[j];
        int fin = arrive + wk / 10;
        if (fin > in.horizon)
            break;
        e += trav + wk;
        t = fin;
        prev = j;
        ++done;
    }
    if (energy_out)
        *energy_out = e;
    return done;
}

static bool seq_ok(const Inst &in, int w, const vector<int> &seq)
{
    return eval_seq(in, w, seq, 0) == static_cast<int>(seq.size());
}

int main(int argc, char **argv)
{
    unsigned int seed = (argc > 1) ? static_cast<unsigned int>(strtoul(argv[1], 0, 10)) : 1u;
    int restarts = (argc > 2) ? atoi(argv[2]) : 300;
    bool notime = (argc > 3) && string(argv[3]) == string("notime");
    bool nofore = (argc > 3) && string(argv[3]) == string("nofore");

    const int MAP_SIZE = 20, NUM_ROBOT = 6, NUM_MAX_TASKS = 16, WALL_DENSITY = 20;
    const int TIME_MAX = MAP_SIZE * 100;
    const int ROBOT_ENERGY = TIME_MAX * 6;
    MAPN = MAP_SIZE;

    srand(seed);
    MAP map(MAP_SIZE, NUM_ROBOT, NUM_MAX_TASKS / 2, NUM_MAX_TASKS, WALL_DENSITY, ROBOT_ENERGY);
    TASKDISPATCHER disp(map, TIME_MAX);
    // robots never move, so the spawn positions depend only on the seed
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

    Inst in;
    in.horizon = notime ? 1000000 : TIME_MAX;
    in.nofore = nofore;
    vector<Coord> wpos;
    for (auto &r : map.get_robots())
    {
        if (r->type == ROBOT::TYPE::DRONE)
            continue;
        in.wtype.push_back(static_cast<int>(r->type));
        in.energy.push_back(ROBOT_ENERGY);
        wpos.push_back(r->get_coord());
    }
    in.nw = static_cast<int>(wpos.size());

    auto &tasks = map.get_tasks();
    in.nt = static_cast<int>(tasks.size());
    vector<Coord> tpos;
    for (int i = 0; i < in.nt; ++i)
    {
        tpos.push_back(tasks[i]->coord);
        in.release.push_back(i < NUM_MAX_TASKS / 2 ? 0 : TIME_MAX / 4 + (i - NUM_MAX_TASKS / 2) * TIME_MAX / NUM_MAX_TASKS);
        vector<int> wk(3, INF);
        for (int ty = 1; ty <= 2; ++ty)
        {
            int c = tasks[i]->get_cost(static_cast<ROBOT::TYPE>(ty));
            int ticks = (c + 9) / 10;
            if (ticks < 1)
                ticks = 1;
            wk[ty] = ticks * 10;
        }
        in.work.push_back(wk);
    }

    in.dstart.assign(in.nw, vector<int>(in.nt, INF));
    for (int w = 0; w < in.nw; ++w)
    {
        vector<int> d = dijkstra(cost, MAP_SIZE, wpos[w], in.wtype[w]);
        for (int j = 0; j < in.nt; ++j)
            in.dstart[w][j] = d[tpos[j].x * MAP_SIZE + tpos[j].y];
    }
    in.dtt.assign(3, vector<vector<int>>(in.nt, vector<int>(in.nt, INF)));
    for (int ty = 1; ty <= 2; ++ty)
        for (int i = 0; i < in.nt; ++i)
        {
            vector<int> d = dijkstra(cost, MAP_SIZE, tpos[i], ty);
            for (int j = 0; j < in.nt; ++j)
                in.dtt[ty][i][j] = d[tpos[j].x * MAP_SIZE + tpos[j].y];
        }

    // ---- search: randomized cheapest insertion + local search ---------------
    vector<vector<int>> best;
    int best_n = -1, greedy_n = -1;
    vector<vector<int>> cur(in.nw);

    auto total = [&](const vector<vector<int>> &s) {
        int c = 0;
        for (int w = 0; w < in.nw; ++w)
            c += static_cast<int>(s[w].size());
        return c;
    };

    for (int rs = 0; rs < restarts; ++rs)
    {
        for (int w = 0; w < in.nw; ++w)
            cur[w].clear();
        vector<char> used(in.nt, 0);
        // insertion phase: cheapest feasible insertion, with a little noise so
        // restarts explore different basins
        for (;;)
        {
            int bw = -1, bj = -1, bp = -1;
            long long bcost = -1;
            for (int j = 0; j < in.nt; ++j)
            {
                if (used[j])
                    continue;
                for (int w = 0; w < in.nw; ++w)
                    for (size_t p = 0; p <= cur[w].size(); ++p)
                    {
                        vector<int> t = cur[w];
                        t.insert(t.begin() + p, j);
                        if (!seq_ok(in, w, t))
                            continue;
                        int e0 = 0, e1 = 0;
                        eval_seq(in, w, cur[w], &e0);
                        eval_seq(in, w, t, &e1);
                        long long c = (e1 - e0);
                        if (rs > 0)
                            c = c * (90 + rand() % 25) / 100;
                        if (bcost < 0 || c < bcost)
                        {
                            bcost = c;
                            bw = w;
                            bj = j;
                            bp = static_cast<int>(p);
                        }
                    }
            }
            if (bw < 0)
                break;
            cur[bw].insert(cur[bw].begin() + bp, bj);
            used[bj] = 1;
        }
        if (rs == 0)
            greedy_n = total(cur);

        // local search: relocate / swap / 2-opt, then retry unplaced tasks
        bool improved = true;
        while (improved)
        {
            improved = false;
            // try to place anything still unassigned after a relocate frees room
            for (int j = 0; j < in.nt && !improved; ++j)
            {
                if (used[j])
                    continue;
                for (int w = 0; w < in.nw && !improved; ++w)
                    for (size_t p = 0; p <= cur[w].size(); ++p)
                    {
                        vector<int> t = cur[w];
                        t.insert(t.begin() + p, j);
                        if (seq_ok(in, w, t))
                        {
                            cur[w] = t;
                            used[j] = 1;
                            improved = true;
                            break;
                        }
                    }
            }
            // relocate one task to another worker/position if it stays feasible
            for (int w = 0; w < in.nw && !improved; ++w)
                for (size_t i = 0; i < cur[w].size() && !improved; ++i)
                {
                    int j = cur[w][i];
                    vector<int> src = cur[w];
                    src.erase(src.begin() + i);
                    for (int w2 = 0; w2 < in.nw && !improved; ++w2)
                    {
                        vector<int> base = (w2 == w) ? src : cur[w2];
                        for (size_t p = 0; p <= base.size(); ++p)
                        {
                            if (w2 == w && p == i)
                                continue;
                            vector<int> t = base;
                            t.insert(t.begin() + p, j);
                            if (!seq_ok(in, w2, t))
                                continue;
                            if (w2 != w && !seq_ok(in, w, src))
                                continue;
                            int e_before = 0, e_after = 0, tmp = 0;
                            eval_seq(in, w, cur[w], &tmp);
                            e_before += tmp;
                            if (w2 != w)
                            {
                                eval_seq(in, w2, cur[w2], &tmp);
                                e_before += tmp;
                            }
                            eval_seq(in, w2, t, &tmp);
                            e_after += tmp;
                            if (w2 != w)
                            {
                                eval_seq(in, w, src, &tmp);
                                e_after += tmp;
                            }
                            if (e_after < e_before)
                            {
                                if (w2 != w)
                                    cur[w] = src;
                                cur[w2] = t;
                                improved = true;
                                break;
                            }
                        }
                    }
                }
            // 2-opt (segment reversal) inside one worker
            for (int w = 0; w < in.nw && !improved; ++w)
                for (size_t a = 0; a + 1 < cur[w].size() && !improved; ++a)
                    for (size_t b = a + 1; b < cur[w].size(); ++b)
                    {
                        vector<int> t = cur[w];
                        reverse(t.begin() + a, t.begin() + b + 1);
                        if (!seq_ok(in, w, t))
                            continue;
                        int e0 = 0, e1 = 0;
                        eval_seq(in, w, cur[w], &e0);
                        eval_seq(in, w, t, &e1);
                        if (e1 < e0)
                        {
                            cur[w] = t;
                            improved = true;
                            break;
                        }
                    }
        }
        int n = total(cur);
        if (n > best_n)
        {
            best_n = n;
            best = cur;
        }
        if (best_n == in.nt)
            break;
    }

    int etot = 0;
    for (int w = 0; w < in.nw; ++w)
    {
        int e = 0;
        eval_seq(in, w, best[w], &e);
        etot += e;
    }
    // how many tasks no worker can physically reach (walled-off pockets)?
    int unreachable = 0;
    for (int j = 0; j < in.nt; ++j)
    {
        bool any = false;
        for (int w = 0; w < in.nw && !any; ++w)
            if (in.dstart[w][j] < INF)
                any = true;
        if (!any)
            ++unreachable;
    }
    cout << seed << "," << best_n << "," << greedy_n << "," << etot << "," << in.nt
         << "," << unreachable << endl;
    return 0;
}
