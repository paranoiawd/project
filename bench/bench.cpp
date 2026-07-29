// Benchmark harness: exact replica of the simulation loop in main.cpp,
// but with (seed, max task cap) taken from argv and a single machine-readable
// CSV result line on stdout.
//
// Usage: ./bench <seed> [num_max_tasks=20]
// Output: seed,cap,created,discovered,completed,exhausted,end_time
#include "../../simulator.h"
#include "../../schedular.h"
#include <cstdlib>
#include <climits>
#include <queue>


// forensic helper: dijkstra over the TRUE cost map for a robot type
static vector<int> true_dijkstra(MAP &map, int n, Coord src, ROBOT::TYPE type)
{
    vector<int> d(n * n, INT_MAX / 4);
    typedef pair<int, int> QE;
    priority_queue<QE, vector<QE>, greater<QE>> pq;
    auto cost = [&](int x, int y) { return map.get_cost({x, y}, type); };
    // get_cost reads known map; use full cost via print… not accessible: use known?  We
    // approximate with known map (what the scheduler itself can see).
    int s = src.x * n + src.y;
    d[s] = 0;
    pq.push(QE(0, s));
    static const int DX[4] = {0, 0, -1, 1}, DY[4] = {1, -1, 0, 0};
    while (!pq.empty())
    {
        QE t = pq.top();
        pq.pop();
        if (t.first != d[t.second])
            continue;
        int ux = t.second / n, uy = t.second % n;
        int cu = cost(ux, uy);
        if (cu < 0)
            cu = (type == ROBOT::TYPE::CATERPILLAR) ? 299 : 448; // unknown estimate
        if (cu == INFINITE)
            continue;
        for (int k = 0; k < 4; ++k)
        {
            int vx = ux + DX[k], vy = uy + DY[k];
            if (vx < 0 || vy < 0 || vx >= n || vy >= n)
                continue;
            int cv = cost(vx, vy);
            if (cv < 0)
                cv = (type == ROBOT::TYPE::CATERPILLAR) ? 299 : 448;
            if (cv == INFINITE)
                continue;
            int w = ((cu / 2 + cv) + 9) / 10 * 10;
            if (t.first + w < d[vx * n + vy])
            {
                d[vx * n + vy] = t.first + w;
                pq.push(QE(t.first + w, vx * n + vy));
            }
        }
    }
    return d;
}

int main(int argc, char **argv)
{
    unsigned int seed = (argc > 1) ? static_cast<unsigned int>(strtoul(argv[1], nullptr, 10)) : 0u;
    const int NUM_MAX_TASKS = (argc > 2) ? atoi(argv[2]) : 20;
    const bool ORACLE = (argc > 3) && string(argv[3]) == string("oracle");
    // "bound": run normally, then solve exactly for the best completion count
    // that was ever available GIVEN the discovery times this run produced,
    // with free optimal routing and no charge for observation.
    const bool BOUND = (argc > 3) && string(argv[3]) == string("bound");
    // "terrain": reveal the MAP but not the tasks.  The scheduler starts knowing
    // every wall and every cell cost, while task discovery still has to be
    // earned exactly as before.  This separates the two things `oracle` conflates
    // -- knowing the layout, and knowing where the work is -- and it is the
    // diagnostic that says whether a smarter scout *route* is reachable at all,
    // since any route planner is only as good as the map it plans on.
    const bool TERRAIN = (argc > 3) && string(argv[3]) == string("terrain");
    const bool NOFORE = getenv("BENCH_BOUND_NOFORE") != nullptr;

    constexpr int MAP_SIZE = 20;
    constexpr int NUM_ROBOT = 6;
    const int NUM_INITIAL_TASKS = NUM_MAX_TASKS / 2;
    constexpr int WALL_DENSITY = 20;
    constexpr int TIME_MAX = MAP_SIZE * 100;
    // BENCH_ENERGY_PCT lets a diagnostic run ask "is energy the binding
    // constraint?" without touching the scheduler.  100 = the real rules.
    const int ROBOT_ENERGY = TIME_MAX * 6 *
                             (getenv("BENCH_ENERGY_PCT") ? atoi(getenv("BENCH_ENERGY_PCT")) : 100) / 100;
    set<Coord> observed_coords;
    set<Coord> updated_coords;

    srand(seed);

    TIMER timer;
    MAP map(MAP_SIZE, NUM_ROBOT, NUM_INITIAL_TASKS, NUM_MAX_TASKS, WALL_DENSITY, ROBOT_ENERGY);
    int time = -1;
    auto &robots = map.get_robots();
    auto &known_cost_map = map.get_known_cost_map();
    auto &known_object_map = map.get_known_object_map();
    auto &active_tasks = map.get_active_tasks();
    Scheduler scheduler;
    TASKDISPATCHER taskdispatcher(map, TIME_MAX);
    vector<Coord> w0pos;
    vector<int> w0type;
    for (auto &r : robots)
        if (r->type != ROBOT::TYPE::DRONE)
        {
            w0pos.push_back(r->get_coord());
            w0type.push_back(static_cast<int>(r->type));
        }

    // instrumentation: per-task spawn/discovery times + affordability windows
    vector<int> spawn_time, disc_time, first_affordable, last_affordable, affordable_ticks;
    vector<vector<int>> cell_last_seen(MAP_SIZE, vector<int>(MAP_SIZE, -1));
    auto track = [&](int t) {
        auto &tasks = map.get_tasks();
        while (spawn_time.size() < tasks.size())
        {
            spawn_time.push_back(t);
            disc_time.push_back(-1);
            first_affordable.push_back(-1);
            last_affordable.push_back(-1);
            affordable_ticks.push_back(0);
        }
        for (auto &tp : active_tasks)
            if (disc_time[tp->id] < 0)
                disc_time[tp->id] = t;
        for (auto &tp : tasks)
            if (tp->is_done() && disc_time[tp->id] < 0)
                disc_time[tp->id] = t;
    };

    if (TERRAIN)
    {
        // Reveal everything, then take the task knowledge back: put every
        // non-wall cell's object state back to UNKNOWN and drop the tasks that
        // the reveal just handed over.  The cost map keeps the true terrain, so
        // the scheduler knows the layout and nothing about the work.  A task on
        // a restored cell is re-discovered normally, because update_coords only
        // fires when the known object lacks the TASK bit.
        set<Coord> all;
        for (int x = 0; x < MAP_SIZE; ++x)
            for (int y = 0; y < MAP_SIZE; ++y)
                all.emplace(x, y);
        map.update_coords(all);
        for (int x = 0; x < MAP_SIZE; ++x)
            for (int y = 0; y < MAP_SIZE; ++y)
                if (known_object_map[x][y] != OBJECT::WALL)
                    known_object_map[x][y] = OBJECT::UNKNOWN;
        active_tasks.clear();
    }

    while (++time < TIME_MAX &&
           robots.size() != map.get_exhausted_robot_num() &&
           map.num_total_task != map.get_completed_task_num())
    {
        taskdispatcher.try_dispatch(time);
        track(time);
        if (ORACLE) // diagnostic: perfect information — every cell observed
        {
            observed_coords.clear();
            for (int x = 0; x < MAP_SIZE; ++x)
                for (int y = 0; y < MAP_SIZE; ++y)
                    observed_coords.emplace(x, y);
            map.update_coords(observed_coords);
        }
        if (time % 50 == 0)
        {
            vector<vector<int>> wd;
            vector<int> weng;
            vector<ROBOT::TYPE> wtype;
            for (auto &r : robots)
            {
                if (r->type == ROBOT::TYPE::DRONE || r->get_status() == ROBOT::STATUS::EXHAUSTED)
                    continue;
                wd.push_back(true_dijkstra(map, MAP_SIZE, r->get_coord(), r->type));
                weng.push_back(r->get_energy());
                wtype.push_back(r->type);
            }
            for (auto &tp : active_tasks)
            {
                if (tp->is_done() || disc_time[tp->id] < 0)
                    continue;
                for (size_t w = 0; w < wd.size(); ++w)
                {
                    int trav = wd[w][tp->coord.x * MAP_SIZE + tp->coord.y];
                    int we = (tp->get_cost(wtype[w]) + 9) / 10 * 10;
                    if (we < 10) we = 10;
                    if (trav < INT_MAX / 8 && trav + we <= weng[w])
                    {
                        if (first_affordable[tp->id] < 0)
                            first_affordable[tp->id] = time;
                        last_affordable[tp->id] = time;
                        affordable_ticks[tp->id] += 50;
                        break;
                    }
                }
            }
        }
        observed_coords = map.observed_coord_by_robot();
        for (auto &c : observed_coords)
            cell_last_seen[c.x][c.y] = time;
        updated_coords = map.update_coords(observed_coords);

        timer.start();
        scheduler.on_info_updated(observed_coords,
                                  updated_coords,
                                  known_cost_map,
                                  known_object_map,
                                  active_tasks,
                                  robots);
        timer.stop();
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
                    timer.start();
                    do_task = scheduler.on_task_reached(observed_coords,
                                                        updated_coords,
                                                        known_cost_map,
                                                        known_object_map,
                                                        active_tasks,
                                                        robots,
                                                        *robot,
                                                        *(task.lock()));
                    timer.stop();
                }

                if (do_task)
                {
                    robot->start_working(task);
                }
                else
                {
                    timer.start();
                    ROBOT::ACTION action = scheduler.idle_action(observed_coords,
                                                                 updated_coords,
                                                                 known_cost_map,
                                                                 known_object_map,
                                                                 active_tasks,
                                                                 robots,
                                                                 *robot);
                    timer.stop();
                    robot->start_moving(action);
                }
            }

            if (status == ROBOT::STATUS::MOVING)
            {
                robot->move();
            }
            else if (status == ROBOT::STATUS::WORKING)
            {
                robot->work();
            }
        }
    }

    int completed = map.get_completed_task_num();
    int discovered = completed + static_cast<int>(active_tasks.size());
    int created = static_cast<int>(map.get_tasks().size());
    int worker_energy = 0, drone_energy = 0;
    for (auto &r : robots)
    {
        if (r->type == ROBOT::TYPE::DRONE)
            drone_energy += r->get_energy();
        else
            worker_energy += r->get_energy();
    }
    int drone_cell_cost = -1;
    for (int x = 0; x < MAP_SIZE && drone_cell_cost < 0; ++x)
        for (int y = 0; y < MAP_SIZE; ++y)
        {
            int c = map.get_cost({x, y}, ROBOT::TYPE::DRONE);
            if (c >= 0 && c != INFINITE)
            {
                drone_cell_cost = c;
                break;
            }
        }
    cout << seed << "," << NUM_MAX_TASKS << "," << created << "," << discovered
         << "," << completed << "," << map.get_exhausted_robot_num() << "," << time
         << "," << worker_energy << "," << drone_energy << "," << drone_cell_cost << endl;

    if (TERRAIN)
        return 0; // the CSV line above is the whole result

    if (BOUND)
    {
        set<Coord> all;
        for (int x = 0; x < MAP_SIZE; ++x)
            for (int y = 0; y < MAP_SIZE; ++y)
                all.emplace(x, y);
        map.update_coords(all);
        const vector<vector<vector<int>>> &cm = map.get_known_cost_map();
        auto &ts = map.get_tasks();
        int nt = static_cast<int>(ts.size());
        int NW = static_cast<int>(w0pos.size());
        vector<Coord> tp;
        vector<int> rel(nt);
        vector<vector<int>> wk(nt, vector<int>(3, INT_MAX / 4));
        for (int i = 0; i < nt; ++i)
        {
            tp.push_back(ts[i]->coord);
            rel[i] = (disc_time[i] < 0) ? INT_MAX / 4 : disc_time[i];
            for (int ty = 1; ty <= 2; ++ty)
            {
                int c = ts[i]->get_cost(static_cast<ROBOT::TYPE>(ty));
                int k = (c + 9) / 10;
                if (k < 1) k = 1;
                wk[i][ty] = k * 10;
            }
        }
        vector<vector<int>> ds(NW, vector<int>(nt, INT_MAX / 4));
        for (int w = 0; w < NW; ++w)
        {
            vector<int> d = true_dijkstra(map, MAP_SIZE, w0pos[w], static_cast<ROBOT::TYPE>(w0type[w]));
            for (int j = 0; j < nt; ++j) ds[w][j] = d[tp[j].x * MAP_SIZE + tp[j].y];
        }
        vector<vector<vector<int>>> dt(3, vector<vector<int>>(nt, vector<int>(nt, INT_MAX / 4)));
        for (int ty = 1; ty <= 2; ++ty)
            for (int i = 0; i < nt; ++i)
            {
                vector<int> d = true_dijkstra(map, MAP_SIZE, tp[i], static_cast<ROBOT::TYPE>(ty));
                for (int j = 0; j < nt; ++j) dt[ty][i][j] = d[tp[j].x * MAP_SIZE + tp[j].y];
            }
        int FULLM = 1 << nt;
        vector<vector<char>> feas(NW, vector<char>(FULLM, 0));
        for (int w = 0; w < NW; ++w)
        {
            int ty = w0type[w];
            vector<vector<pair<int,int>>> par((size_t)FULLM * nt);
            auto addp = [&](size_t key, int e, int t) {
                auto &v = par[key];
                for (auto &q : v) if (q.first <= e && q.second <= t) return;
                for (size_t i = 0; i < v.size();) { if (v[i].first >= e && v[i].second >= t) v.erase(v.begin()+i); else ++i; }
                v.push_back(make_pair(e, t));
            };
            for (int j = 0; j < nt; ++j)
            {
                if (ds[w][j] >= INT_MAX/8 || rel[j] >= INT_MAX/8) continue;
                int e = ds[w][j] + wk[j][ty];
                if (e > ROBOT_ENERGY) continue;
                int dep0 = NOFORE ? rel[j] : 0;
                int st2 = max(dep0 + ds[w][j] / 10, rel[j]);
                int t = st2 + wk[j][ty] / 10;
                if (t > TIME_MAX) continue;
                addp((size_t)(1 << j) * nt + j, e, t);
                feas[w][1 << j] = 1;
            }
            vector<int> ord;
            for (int S = 1; S < FULLM; ++S) ord.push_back(S);
            sort(ord.begin(), ord.end(), [](int a, int b){ return __builtin_popcount(a) < __builtin_popcount(b); });
            for (int S : ord)
                for (int last = 0; last < nt; ++last)
                {
                    if (!((S >> last) & 1)) continue;
                    auto cur = par[(size_t)S * nt + last];
                    if (cur.empty()) continue;
                    feas[w][S] = 1;
                    for (int j = 0; j < nt; ++j)
                    {
                        if ((S >> j) & 1) continue;
                        if (dt[ty][last][j] >= INT_MAX/8 || rel[j] >= INT_MAX/8) continue;
                        for (auto &c : cur)
                        {
                            int e = c.first + dt[ty][last][j] + wk[j][ty];
                            if (e > ROBOT_ENERGY) continue;
                            int dep = c.second;
                            if (NOFORE && dep < rel[j]) dep = rel[j];
                            int st2 = max(dep + dt[ty][last][j] / 10, rel[j]);
                            int t = st2 + wk[j][ty] / 10;
                            if (t > TIME_MAX) continue;
                            addp((size_t)(S | (1 << j)) * nt + j, e, t);
                        }
                    }
                }
        }
        vector<signed char> dp(FULLM, -1), ndp(FULLM, -1);
        dp[0] = 0;
        for (int w = 0; w < NW; ++w)
        {
            ndp = dp;
            for (int S = 0; S < FULLM; ++S)
            {
                if (dp[S] < 0) continue;
                int rest = (FULLM - 1) & ~S;
                for (int T = rest; T; T = (T - 1) & rest)
                {
                    if (!feas[w][T]) continue;
                    int v = dp[S] + __builtin_popcount(T);
                    if (v > ndp[S | T]) ndp[S | T] = (signed char)v;
                }
            }
            dp.swap(ndp);
        }
        int bnd = 0;
        for (int S = 0; S < FULLM; ++S) if (dp[S] > bnd) bnd = dp[S];
        cout << "BOUND," << seed << "," << completed << "," << discovered << "," << bnd << endl;
        return 0;
    }

    if (argc > 3) // verbose dump for failure analysis
    {
        // per-robot economics: energy spent vs tasks completed
        {
            vector<int> done_by(robots.size(), 0);
            for (auto &tp : map.get_tasks())
                if (tp->is_done() && tp->get_assigned_robot_id() >= 0)
                    done_by[tp->get_assigned_robot_id()]++;
            cout << "robot economics (id type spent done spent/task):" << endl;
            for (auto &r : robots)
            {
                int spent = ROBOT_ENERGY - r->get_energy();
                cout << "  R" << r->id << " " << to_string(r->type)[0]
                     << " spent=" << spent << " done=" << done_by[r->id]
                     << " per=" << (done_by[r->id] ? spent / done_by[r->id] : -1) << endl;
            }
        }
        track(time);
        int unknown_cells = 0;
        auto &kom = map.get_known_object_map();
        for (auto &col : kom)
            for (auto &o : col)
                if (o == OBJECT::UNKNOWN)
                    ++unknown_cells;
        {
            // Of the cells never observed, how many were actually open ground?
            // A wall nobody ever saw is not a missed opportunity.  The known map
            // reports -1 for an unobserved cell, so the true map has to be
            // revealed first, after recording which cells were still unknown.
            vector<Coord> unk;
            for (int x = 0; x < MAP_SIZE; ++x)
                for (int y = 0; y < MAP_SIZE; ++y)
                    if (kom[x][y] == OBJECT::UNKNOWN)
                        unk.emplace_back(x, y);
            set<Coord> all;
            for (int x = 0; x < MAP_SIZE; ++x)
                for (int y = 0; y < MAP_SIZE; ++y)
                    all.emplace(x, y);
            map.update_coords(all);
            int unk_open = 0;
            for (size_t q = 0; q < unk.size(); ++q)
                if (map.get_cost(unk[q], ROBOT::TYPE::CATERPILLAR) != INFINITE)
                    ++unk_open;
            cout << "unknown cells at end: " << unknown_cells << "/" << MAP_SIZE * MAP_SIZE
                 << " (non-wall: " << unk_open << ")" << endl;
        }
        {
            int hole = 0, timing = 0;
            for (auto &tp : map.get_tasks())
            {
                if (disc_time[tp->id] >= 0)
                    continue;
                int ls = cell_last_seen[tp->coord.x][tp->coord.y];
                if (ls < 0)
                    ++hole;
                else if (ls < spawn_time[tp->id])
                    ++timing;
                cout << "  MISS T" << tp->id << " spawn=" << spawn_time[tp->id]
                     << " cell_last_seen=" << ls << " " << tp->coord << endl;
            }
            cout << "undiscovered: coverage_hole=" << hole << " timing_miss=" << timing << endl;
            int A = 0, B = 0, C = 0;
            for (auto &tp : map.get_tasks())
            {
                if (tp->is_done() || disc_time[tp->id] < 0)
                    continue;
                if (disc_time[tp->id] > 1600)
                    ++C; // discovered too late
                else if (first_affordable[tp->id] < 0)
                    ++A; // never affordable for anyone after discovery
                else
                    ++B; // was affordable but never served
            }
            cout << "undone classification: A_never_affordable=" << A
                 << " B_affordable_unserved=" << B << " C_late_discovery=" << C << endl;
        }
        cout << "task id: spawn_t disc_t done aff[first,last,dur] coord cost(cat/wheel)" << endl;
        auto &tasks = map.get_tasks();
        for (auto &tp : tasks)
            cout << "  T" << tp->id << ": " << spawn_time[tp->id] << " " << disc_time[tp->id]
                 << " " << (tp->is_done() ? "DONE" : "----")
                 << " [" << first_affordable[tp->id] << "," << last_affordable[tp->id]
                 << "," << affordable_ticks[tp->id] << "] " << tp->coord
                 << " " << tp->get_cost(ROBOT::TYPE::CATERPILLAR)
                 << "/" << tp->get_cost(ROBOT::TYPE::WHEEL) << endl;
        map.print_robot_summary();
        map.print_known_object_map();
    }
    return 0;
}
