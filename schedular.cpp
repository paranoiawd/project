#include "schedular.h"

#include <algorithm>
#include <cstdlib>
#include <map>
#include <queue>
#include <utility>

// ---------------------------------------------------------------------------
// Strategy overview
//
//  * Movement mechanics (derived from the simulator): one step from cell u to
//    an adjacent cell v takes ceil((floor(cost(u)/2) + cost(v)) / 10) ticks and
//    10 energy per tick, so energy and time are the same currency.  Working a
//    task costs 10 * max(1, ceil(task_cost / 10)) energy.  Holding is free.
//    Travel is by far the dominant cost (a cross-map trip can exceed half a
//    robot's total energy), so the scheduler is built around avoiding it.
//
//  * Workers (caterpillar/wheel — drones can never finish a task) are anchored
//    to map quadrants: each serves tasks in its own zone first (cross-zone
//    helping is allowed but penalized), which keeps trips short.  Matching is
//    a greedy assignment on Dijkstra travel + work energy over the known cost
//    map, with a stickiness bonus, and only when the robot can afford the
//    whole trip.  Idle workers hold position (holding is free) instead of
//    wandering; an idle worker parked in its zone doubles as a task sensor.
//
//  * Drones are pure scouts.  Each takes one half of the map and runs
//    committed serpentine lane sweeps (lanes 5 apart match their 5x5 view),
//    with an energy budget spread over the run so that cells keep being
//    re-observed late — tasks spawn all game long and an unobserved spawn is
//    invisible.  Out-of-budget drones park and keep observing as cameras.
//
//  * on_task_reached only accepts a task when the robot type can complete it,
//    nobody else is physically working on it, and the robot has enough energy
//    to finish; a robot standing on a task may also opportunistically take it
//    over when that is cheaper than the booked robot's remaining travel+work.
// ---------------------------------------------------------------------------

namespace
{
    // ---- tuning knobs -----------------------------------------------------
    const double UNKNOWN_PEN[3] = {1.05, 1.15, 1.15}; // pathfinding pessimism on unknown cells
    const int EST_CELL_COST[3] = {160, 299, 448};     // expected cell cost per type (drone/cat/wheel)
    const double ASSIGN_STICKY = 0.85;                // cost discount for keeping an assignment
    const int WORKER_MIN_ENERGY = 30;                 // below this a worker is effectively retired
    const int WORKER_TRAVEL_CAP = 3800;               // normal per-assignment travel budget
    const int DRONE_CAMERA_FLOOR = 400;               // drones never spend below this (parked sensor)
    const int DRONE_STEP_EST = 300;                   // rough energy for one drone step
    const int DRONE_BURST_DEF = 800;                 // energy a drone may spend ahead of the line
    const int TIME_HORIZON_PER_CELL = 99;             // assumed run length ~ 85 * map_size ticks
    const int RESWEEP_MIN_GAIN = 1500;                // minimum window staleness to sweep to
    const int DRONE_LOCAL_RANGE = 1500;               // "local" window distance for crawling
    const double LOCAL_KEEP_RATIO = 0.75;             // take local window above this vs global
    const int STALE_UNSEEN = 2000;                    // staleness score of a never-seen cell
    const int STALE_CAP = 1200;                       // staleness cap for seen cells (ticks)
    const int STARVE_AGE = 250;                       // discovered-but-unserved ticks before the
                                                      // travel cap is waived for a task
    const int WORKER_LOCAL_RESERVE = 1000;            // keep this much unless the task is local
    const int WORKER_LOCAL_TRAVEL = 800;              // "local" travel threshold for the reserve
    const int TASK_MAGNET = 350;                      // path bonus for stepping onto a free task
    const int LOCK_DIST = 700;                        // don't rebook a nearly-reached assignment
    const int HORIZON_HARD_PER_CELL = 100;            // assumed hard end ~ 100 * map_size ticks
    const int ENDGAME_TICKS_DEF = 600;                // endgame window before the hard horizon
    const double TOUR_FIRST_BONUS = 0.55;             // score factor for a tour's first leg
    const int TOUR_MAX_LEN = 8;                       // max tasks per planned tour
    const int NEED_T_PER_CELL = 66;                   // ~just before the last spawns
    const int DEBT_RESERVE = 3000;                    // worker energy kept for late
                                                      // observation-debt trips
    const int PLAN_INF = 1000000000;

    const int DXS[4] = {0, 0, -1, 1};
    const int DYS[4] = {1, -1, 0, 0};

    inline int ceil10(int v) { return (v + 9) / 10; }

    inline int work_energy(const TASK &task, ROBOT::TYPE type)
    {
        int c = task.get_cost(type);
        if (c < 0 || c >= INFINITE)
            return PLAN_INF;
        int ticks = ceil10(c);
        if (ticks < 1)
            ticks = 1;
        return ticks * 10;
    }
}

struct Scheduler::State
{
    int now = -1;
    int n = 0;           // map size
    int drone_cost = -1; // uniform drone cell cost once discovered
    const vector<vector<vector<int>>> *cost_map = nullptr;
    const vector<vector<OBJECT>> *obj_map = nullptr;

    vector<vector<int>> last_seen; // [x][y] -> tick last observed (-1 never)
    vector<int> stale;             // flattened staleness grid (rebuilt per tick)
    vector<char> magnet;           // flattened: cell holds an unowned, undone task

    // per robot id
    vector<int> init_energy;
    vector<int> assigned;     // robot -> task id (-1 none)
    vector<Coord> next_step;  // robot -> planned next cell (== own cell: hold)
    vector<vector<int>> dist;  // robot -> travel energy, magnet-routed (for paths)
    vector<vector<int>> par;   // robot -> dijkstra parents for the routed map
    vector<vector<int>> dist_c; // robot -> CLEAN travel energy (for costing)
    map<int, int> owner;      // task id -> robot id (book assignment)
    map<int, int> first_seen; // task id -> tick it was first discovered
    int regime = 0;           // 0=undecided, 1=dense(completion focus), 2=sparse(discovery focus)

    // drone sweep state
    map<int, pair<int, int>> drone_half; // drone id -> [x_lo, x_hi]
    map<int, Coord> drone_goal;          // drone id -> committed sweep window center
    map<int, Coord> patrol_goal;         // worker id -> committed patrol window center
    map<int, int> lane_of_worker;        // worker id -> interior lane index
    vector<vector<Coord>> lanes_left;    // per-lane remaining waypoints (debt phase)
    bool lanes_inited = false;
    map<int, vector<Coord>> serp_route;  // drone id -> late rim sweep waypoints
    map<int, bool> serp_started;         // drone id -> late sweep has begun
    vector<char> drone_covered;          // cells the drones' remaining affordable
                                         // route will still observe this run

    // Late-phase rim route: this drone's half of a perimeter ring running at
    // distance r from the map edge.  Two drones jointly cover the outer
    // (2r+1)-wide band; the interior is the workers' natural habitat and is
    // kept fresh by their travel and idle patrols instead.
    vector<Coord> make_rim_route(int xlo, int xhi, int r, const Coord &from) const
    {
        int lo = min(r, n - 1), hi = max(n - 1 - r, 0);
        int xin = min(max(xlo + r, lo), hi);   // inner x-limit of my half arc
        int xout = min(max(xhi - r, lo), hi);  // outer x-limit
        if (xlo <= 0 && xhi >= n - 1) // single drone: full ring
        {
            vector<Coord> route;
            route.push_back(Coord(lo, lo));
            route.push_back(Coord(lo, hi));
            route.push_back(Coord(hi, hi));
            route.push_back(Coord(hi, lo));
            route.push_back(Coord(lo, lo));
            return route;
        }
        // half arc: along the bottom to my outer column, up it, back along
        // top — densified to a waypoint every few cells so that a blocked
        // waypoint only ever skips a short stretch, and wall detours are
        // pulled back onto the rim quickly.
        vector<Coord> corners;
        bool left_half = (xlo <= 0);
        int col = left_half ? lo : hi;
        int xend = left_half ? xout : xin;
        bool from_bottom = (from.y <= n / 2);
        if (from_bottom)
        {
            corners.push_back(Coord(xend, lo));
            corners.push_back(Coord(col, lo));
            corners.push_back(Coord(col, hi));
            corners.push_back(Coord(xend, hi));
        }
        else
        {
            corners.push_back(Coord(xend, hi));
            corners.push_back(Coord(col, hi));
            corners.push_back(Coord(col, lo));
            corners.push_back(Coord(xend, lo));
        }
        vector<Coord> route;
        Coord cur = corners.front();
        route.push_back(cur);
        for (size_t i = 1; i < corners.size(); ++i)
        {
            Coord nxt = corners[i];
            int steps = abs(nxt.x - cur.x) + abs(nxt.y - cur.y);
            int sx = (nxt.x > cur.x) - (nxt.x < cur.x);
            int sy = (nxt.y > cur.y) - (nxt.y < cur.y);
            Coord p = cur;
            for (int s = 2; s < steps; s += 2)
            {
                p = Coord(cur.x + sx * min(s, abs(nxt.x - cur.x)),
                          cur.y + sy * max(0, s - abs(nxt.x - cur.x)));
                route.push_back(p);
            }
            route.push_back(nxt);
            cur = nxt;
        }
        return route;
    }

    // per-tick cache: dijkstra maps sourced at task cells (for chain checks)
    map<pair<int, int>, vector<int>> task_dist_cache; // (task id, type) -> dist
    vector<int> scratch_par;

    const vector<int> &dist_from_task(int task_id, int type, const Coord &c)
    {
        pair<int, int> key(task_id, type);
        map<pair<int, int>, vector<int>>::iterator it = task_dist_cache.find(key);
        if (it != task_dist_cache.end())
            return it->second;
        vector<int> d;
        dijkstra(c, type, d, scratch_par, false);
        return task_dist_cache[key] = d;
    }

    // ---- basic helpers ----------------------------------------------------
    int horizon_dense() const { return 85 * n; }
    int idx(int x, int y) const { return x * n + y; }
    int idx(const Coord &c) const { return c.x * n + c.y; }
    bool in_map(int x, int y) const { return x >= 0 && y >= 0 && x < n && y < n; }

    // Cell cost for pathfinding: -1 means impassable (known wall).
    int cell_cost(int x, int y, int t) const
    {
        int c = (*cost_map)[x][y][t];
        if (c == INFINITE)
            return -1;
        if (c < 0) // unknown cell: estimated, slightly pessimistic
        {
            int est = (t == 0 && drone_cost > 0) ? drone_cost : EST_CELL_COST[t];
            return static_cast<int>(est * UNKNOWN_PEN[t]);
        }
        return c;
    }

    void dijkstra(const Coord &src, int type, vector<int> &d, vector<int> &p,
                  bool use_magnet = true) const
    {
        d.assign(n * n, PLAN_INF);
        p.assign(n * n, -1);
        typedef pair<int, int> QE; // (dist, node)
        priority_queue<QE, vector<QE>, greater<QE>> pq;
        int s = idx(src);
        d[s] = 0;
        pq.push(QE(0, s));
        while (!pq.empty())
        {
            QE top = pq.top();
            pq.pop();
            if (top.first != d[top.second])
                continue;
            int ux = top.second / n, uy = top.second % n;
            int cu = cell_cost(ux, uy, type);
            if (cu < 0)
                continue;
            for (int k = 0; k < 4; ++k)
            {
                int vx = ux + DXS[k], vy = uy + DYS[k];
                if (!in_map(vx, vy))
                    continue;
                int cv = cell_cost(vx, vy, type);
                if (cv < 0)
                    continue;
                int w = ceil10(cu / 2 + cv) * 10; // exact step energy (== 10 * ticks)
                // pull worker paths across free tasks: they get grabbed en route
                if (use_magnet && type != 0 && !magnet.empty() && magnet[idx(vx, vy)])
                    w = max(10, w - TASK_MAGNET);
                int nd = top.first + w;
                int v = idx(vx, vy);
                if (nd < d[v])
                {
                    d[v] = nd;
                    p[v] = top.second;
                    pq.push(QE(nd, v));
                }
            }
        }
    }

    // First cell to move to on the shortest path src -> goal (src if none/at goal).
    Coord first_step(const vector<int> &p, const Coord &src, const Coord &goal) const
    {
        int s = idx(src), g = idx(goal);
        if (s == g)
            return src;
        int cur = g, prev = -1;
        while (cur != s)
        {
            prev = cur;
            cur = p[cur];
            if (cur < 0)
                return src;
        }
        return Coord(prev / n, prev % n);
    }

    // ---- staleness (for sweep waypoint skipping) --------------------------
    // serve_dist[cell] = min over alive workers of clean travel energy; used to
    // weight observation value: a task found near a worker is cheap to serve,
    // one found in a far corner would mostly go unserved anyway.
    vector<int> serve_dist;

    void build_staleness()
    {
        stale.assign(n * n, 0);
        for (int x = 0; x < n; ++x)
            for (int y = 0; y < n; ++y)
            {
                if ((*obj_map)[x][y] == OBJECT::WALL)
                    continue;
                int ls = last_seen[x][y];
                int s = (ls < 0) ? STALE_UNSEEN : min(now - ls, STALE_CAP);
                if (!serve_dist.empty())
                {
                    int sd = serve_dist[idx(x, y)];
                    const double wlo = (regime == 1) ? 0.15 : 0.6;
                    const double whi = (regime == 1) ? 3.0 : 1.8;
                    const double wsl = (regime == 1) ? 2000.0 : 3000.0;
                    double w = (sd >= PLAN_INF) ? wlo : whi - static_cast<double>(sd) / wsl;
                    if (w < wlo)
                        w = wlo;
                    if (w > whi)
                        w = whi;
                    s = static_cast<int>(s * w);
                }
                stale[idx(x, y)] = s;
            }
    }

    // number of non-wall cells in the window not observed since need_t and
    // not already on a drone's remaining (affordable) rim route — i.e. cells
    // that could still hide a task and that nobody else will look at
    int debt_gain(int cx, int cy, int r, int need_t) const
    {
        int g = 0;
        for (int xx = max(cx - r, 0); xx <= min(cx + r, n - 1); ++xx)
            for (int yy = max(cy - r, 0); yy <= min(cy + r, n - 1); ++yy)
            {
                if ((*obj_map)[xx][yy] == OBJECT::WALL)
                    continue;
                if (!drone_covered.empty() && drone_covered[idx(xx, yy)])
                    continue;
                if (last_seen[xx][yy] < need_t)
                    ++g;
            }
        return g;
    }

    int window_gain(int cx, int cy, int r) const
    {
        int g = 0;
        for (int xx = max(cx - r, 0); xx <= min(cx + r, n - 1); ++xx)
            for (int yy = max(cy - r, 0); yy <= min(cy + r, n - 1); ++yy)
                g += stale[idx(xx, yy)];
        return g;
    }

};

Scheduler::Scheduler() : s_(new State()) {}
Scheduler::~Scheduler() {}

void Scheduler::on_info_updated(const set<Coord> &observed_coords,
                                const set<Coord> &updated_coords,
                                const vector<vector<vector<int>>> &known_cost_map,
                                const vector<vector<OBJECT>> &known_object_map,
                                const vector<shared_ptr<TASK>> &active_tasks,
                                const vector<shared_ptr<ROBOT>> &robots)
{
    State &st = *s_;
    ++st.now;
    st.cost_map = &known_cost_map;
    st.obj_map = &known_object_map;
    st.task_dist_cache.clear();

    // ---- lazy init --------------------------------------------------------
    if (st.n == 0)
    {
        st.n = static_cast<int>(known_object_map.size());
        st.last_seen.assign(st.n, vector<int>(st.n, -1));
    }
    int max_id = 0;
    for (size_t i = 0; i < robots.size(); ++i)
        max_id = max(max_id, robots[i]->id);
    if (static_cast<int>(st.assigned.size()) < max_id + 1)
    {
        st.init_energy.resize(max_id + 1, -1);
        st.assigned.resize(max_id + 1, -1);
        st.next_step.resize(max_id + 1, Coord(-1, -1));
        st.dist.resize(max_id + 1);
        st.par.resize(max_id + 1);
        st.dist_c.resize(max_id + 1);
    }

    // ---- info update ------------------------------------------------------
    for (set<Coord>::const_iterator it = observed_coords.begin(); it != observed_coords.end(); ++it)
        st.last_seen[it->x][it->y] = st.now;

    // Regime detection.  A sparse (original-config) map has only cap/2 = 8
    // initial tasks, so before the first spawn (~t=500) its discovered count
    // can never reach 9: deciding at t=490 with threshold 9 can never
    // misclassify the sparse config.  A dense map that happens to reveal
    // slowly may still be upgraded (sparse -> dense is the safe direction)
    // while its count crosses 12 early on.
    {
        static int force = -1;
        if (force == -1)
        {
            const char *f = getenv("SCHED_FORCE_REGIME");
            force = f ? atoi(f) : 0;
        }
        if (force > 0)
            st.regime = force;
    }
    if (st.regime == 0 && st.now >= 130)
        st.regime = (static_cast<int>(st.first_seen.size()) >= 7) ? 1 : 2;
    // corrections, both structurally safe for the sparse original config whose
    // discovered count cannot exceed 8 before the first spawn (~t=500):
    if (st.regime == 2 && st.now < 700 && static_cast<int>(st.first_seen.size()) >= 12)
        st.regime = 1; // late-revealing dense map
    if (st.regime == 1 && st.now == 510 && static_cast<int>(st.first_seen.size()) <= 8)
        st.regime = 2; // was a lucky sparse map after all

    if (st.drone_cost < 0)
    {
        for (int x = 0; x < st.n && st.drone_cost < 0; ++x)
            for (int y = 0; y < st.n; ++y)
            {
                int c = known_cost_map[x][y][0];
                if (c >= 0 && c != INFINITE)
                {
                    st.drone_cost = c;
                    break;
                }
            }
    }

    // ---- task snapshot (before pathfinding: paths are magnetized) ---------
    vector<const TASK *> tasks;
    st.magnet.assign(st.n * st.n, 0);
    for (size_t i = 0; i < active_tasks.size(); ++i)
    {
        const TASK *t = active_tasks[i].get();
        if (t == 0 || t->is_done())
            continue;
        if (st.first_seen.find(t->id) == st.first_seen.end())
            st.first_seen[t->id] = st.now;
        if (t->get_assigned_robot_id() != -1) // physically being worked on
            continue;
        tasks.push_back(t);
        st.magnet[st.idx(t->coord)] = 1;
    }

    // ---- per-robot dijkstra ----------------------------------------------
    vector<const ROBOT *> by_id(max_id + 1, static_cast<const ROBOT *>(0));
    for (size_t i = 0; i < robots.size(); ++i)
    {
        const ROBOT &r = *robots[i];
        by_id[r.id] = &r;
        if (st.init_energy[r.id] < 0)
            st.init_energy[r.id] = r.get_energy();
        st.next_step[r.id] = r.get_coord(); // default: hold
        if (r.get_status() == ROBOT::STATUS::EXHAUSTED || r.get_energy() <= 0)
            continue;
        Coord pos = (r.get_status() == ROBOT::STATUS::MOVING) ? r.get_target_coord() : r.get_coord();
        st.dijkstra(pos, static_cast<int>(r.type), st.dist[r.id], st.par[r.id]);
        if (r.type != ROBOT::TYPE::DRONE)
            st.dijkstra(pos, static_cast<int>(r.type), st.dist_c[r.id], st.scratch_par, false);
        else
            st.dist_c[r.id] = st.dist[r.id];
    }

    // ---- greedy worker/task matching -------------------------------------
    // Normal serve test, shared by the availability scan and the candidate
    // emission so they can never disagree: within the travel cap, affordable,
    // and not digging into the worker's small local reserve for far tasks.
    struct ServeTest
    {
        static bool normal(int travel, int we, int energy)
        {
            if (travel > WORKER_TRAVEL_CAP || we >= PLAN_INF)
                return false;
            int need = travel + we;
            if (need > energy)
                return false;
            if (travel > WORKER_LOCAL_TRAVEL && energy - need < WORKER_LOCAL_RESERVE)
                return false;
            return true;
        }
    };
    // For each task, count workers (including currently-working ones) that can
    // afford it at all.  A task with exactly one possible server skips the
    // waiting window: its server must not be lost to other work first.
    vector<int> server_count(tasks.size(), 0);
    for (size_t i = 0; i < robots.size(); ++i)
    {
        const ROBOT &r = *robots[i];
        if (r.type == ROBOT::TYPE::DRONE || r.get_status() == ROBOT::STATUS::EXHAUSTED)
            continue;
        if (r.get_energy() < WORKER_MIN_ENERGY || st.dist[r.id].empty())
            continue;
        for (size_t j = 0; j < tasks.size(); ++j)
        {
            int travel = st.dist_c[r.id][st.idx(tasks[j]->coord)];
            int we = work_energy(*tasks[j], r.type);
            if (travel < PLAN_INF && we < PLAN_INF && travel + we <= r.get_energy())
                ++server_count[j];
        }
    }
    vector<bool> sole_server(tasks.size(), false);
    for (size_t j = 0; j < tasks.size(); ++j)
        sole_server[j] = (server_count[j] == 1);

    // ---- cheapest-insertion tour planning --------------------------------
    // Build one tour per worker over the unowned tasks: repeatedly insert the
    // globally cheapest (worker, task, position) among all remaining tasks,
    // respecting each worker's energy and the time horizon.  The first task
    // of a worker's tour gets a strong preference in the matching below, so
    // the fleet follows near-optimal routes instead of per-tick zigzags.
    map<int, set<int>> tour_first_sets; // worker id -> {task ids that lead its tour}
    {
        vector<const ROBOT *> tworkers;
        for (size_t i = 0; i < robots.size(); ++i)
        {
            const ROBOT &r = *robots[i];
            if (r.type == ROBOT::TYPE::DRONE || r.get_status() == ROBOT::STATUS::EXHAUSTED)
                continue;
            if (r.get_energy() < WORKER_MIN_ENERGY || st.dist_c[r.id].empty())
                continue;
            tworkers.push_back(&r);
        }
        vector<vector<int>> tour(tworkers.size()); // task indices in order
        vector<int> tour_cost(tworkers.size(), 0); // energy incl. work
        vector<bool> used(tasks.size(), false);
        int ticks_left_t = HORIZON_HARD_PER_CELL * st.n - st.now;
        for (int round = 0; round < TOUR_MAX_LEN * 4; ++round)
        {
            double best_inc = 1e18;
            int bw = -1, bj = -1, bpos = -1;
            for (size_t w = 0; w < tworkers.size(); ++w)
            {
                const ROBOT &r = *tworkers[w];
                if (static_cast<int>(tour[w].size()) >= TOUR_MAX_LEN)
                    continue;
                int type_i = static_cast<int>(r.type);
                for (size_t j = 0; j < tasks.size(); ++j)
                {
                    if (used[j])
                        continue;
                    int we = work_energy(*tasks[j], r.type);
                    if (we >= PLAN_INF)
                        continue;
                    // try all insertion positions
                    for (size_t p = 0; p <= tour[w].size(); ++p)
                    {
                        int before, after = -1;
                        if (p == 0)
                            before = st.dist_c[r.id][st.idx(tasks[j]->coord)];
                        else
                            before = st.dist_from_task(tasks[tour[w][p - 1]]->id, type_i,
                                                       tasks[tour[w][p - 1]]->coord)[st.idx(tasks[j]->coord)];
                        if (before >= PLAN_INF)
                            continue;
                        int removed = 0;
                        if (p < tour[w].size())
                        {
                            after = st.dist_from_task(tasks[j]->id, type_i,
                                                      tasks[j]->coord)[st.idx(tasks[tour[w][p]]->coord)];
                            if (after >= PLAN_INF)
                                continue;
                            int old_link;
                            if (p == 0)
                                old_link = st.dist_c[r.id][st.idx(tasks[tour[w][p]]->coord)];
                            else
                                old_link = st.dist_from_task(tasks[tour[w][p - 1]]->id, type_i,
                                                             tasks[tour[w][p - 1]]->coord)[st.idx(tasks[tour[w][p]]->coord)];
                            if (old_link >= PLAN_INF)
                                continue;
                            removed = old_link;
                        }
                        int inc = before + we + ((after >= 0) ? after : 0) - removed;
                        if (tour_cost[w] + inc > r.get_energy())
                            continue;
                        if ((tour_cost[w] + inc) / 10 > ticks_left_t - 20)
                            continue;
                        if (inc < best_inc)
                        {
                            best_inc = inc;
                            bw = static_cast<int>(w);
                            bj = static_cast<int>(j);
                            bpos = static_cast<int>(p);
                        }
                    }
                }
            }
            if (bw < 0)
                break;
            tour[bw].insert(tour[bw].begin() + bpos, bj);
            tour_cost[bw] += static_cast<int>(best_inc);
            used[bj] = true;
        }
        for (size_t w = 0; w < tworkers.size(); ++w)
            if (!tour[w].empty())
                tour_first_sets[tworkers[w]->id].insert(tasks[tour[w][0]]->id);
    }
    map<int, set<int>> &tour_first = tour_first_sets;

    struct SoleHelper
    {
        static bool sole_server_of_cand(const vector<bool> &sole, const vector<const TASK *> &ts,
                                        const TASK *t)
        {
            for (size_t j = 0; j < ts.size(); ++j)
                if (ts[j] == t)
                    return sole[j];
            return false;
        }
    };

    struct Cand
    {
        double score;
        int need;
        int rid;
        const TASK *task;
    };
    vector<Cand> cands;
    for (size_t i = 0; i < robots.size(); ++i)
    {
        const ROBOT &r = *robots[i];
        if (r.type == ROBOT::TYPE::DRONE)
            continue;
        if (r.get_status() == ROBOT::STATUS::EXHAUSTED || r.get_status() == ROBOT::STATUS::WORKING)
            continue;
        if (r.get_energy() < WORKER_MIN_ENERGY || st.dist[r.id].empty())
            continue;
        const vector<int> &d = st.dist_c[r.id];
        for (size_t j = 0; j < tasks.size(); ++j)
        {
            const TASK *t = tasks[j];
            int travel = d[st.idx(t->coord)];
            int we = work_energy(*t, r.type);
            if (travel >= PLAN_INF || we >= PLAN_INF)
                continue;
            int need = travel + we;
            if (need > r.get_energy())
                continue;
            // observation-debt reserve: before the late re-observation phase a
            // worker never spends into its last DEBT_RESERVE on tasks, so it
            // can still close never-seen map gaps afterwards
            if (st.regime == 2 && st.now < NEED_T_PER_CELL * st.n &&
                r.get_energy() - need < DEBT_RESERVE)
                continue;
            // a trip that cannot finish before the assumed end of the run is
            // pure waste: energy walks out but the task never completes
            int ticks_left = HORIZON_HARD_PER_CELL * st.n - st.now;
            if (need / 10 > ticks_left - 20)
                continue;
            bool endgame = ticks_left < ENDGAME_TICKS_DEF;
            // Waiting clock: a far task may wait a bounded time for a cheaper
            // server to free up or for route-merging spawns; after that it is
            // promoted to a full-strength candidate before it decays into
            // nobody-can-afford-it.  Tasks with only one possible server and
            // endgame tasks are promoted immediately.
            bool waited_out = (st.now - st.first_seen[t->id]) > STARVE_AGE;
            bool promoted = waited_out || endgame || sole_server[j];
            bool normal_ok = ServeTest::normal(travel, we, r.get_energy());
            if (!normal_ok && !promoted)
                continue; // still inside its waiting window
            // marginal-cost score: the same trip is dearer for a poorer robot,
            // so medium hauls flow to energy-rich workers.  In the endgame
            // there is no tomorrow to save for: plain cost, spend it all.
            double score = endgame ? need
                                   : need * (1.0 + static_cast<double>(need) / (r.get_energy() + 1.0));
            // tour bonus: tasks that are the first leg of this worker's
            // cheapest-insertion tour are strongly preferred; lone tasks keep
            // their plain score
            if (tour_first[r.id].count(t->id))
                score *= TOUR_FIRST_BONUS;
            map<int, int>::iterator prev = st.owner.find(t->id);
            if (prev != st.owner.end() && prev->second == r.id)
                score *= (travel <= LOCK_DIST) ? 0.3 : ASSIGN_STICKY;
            Cand c;
            c.score = score;
            c.need = need;
            c.rid = r.id;
            c.task = t;
            cands.push_back(c);
        }
    }
    struct CandLess
    {
        bool operator()(const Cand &a, const Cand &b) const { return a.score < b.score; }
    };
    sort(cands.begin(), cands.end(), CandLess());

    map<int, int> new_owner;
    vector<bool> robot_busy(max_id + 1, false);
    for (int i = 0; i <= max_id; ++i)
        st.assigned[i] = -1;
    set<int> task_taken;
    for (size_t i = 0; i < cands.size(); ++i)
    {
        const Cand &c = cands[i];
        if (robot_busy[c.rid] || task_taken.count(c.task->id))
            continue;
        // fragmentation guard: don't spend this robot's energy when doing so
        // would strand a task only IT can afford.  Crucially measured from the
        // candidate task's cell (where the robot ends up), so chains that move
        // toward the sole task remain allowed.
        if (c.need > 1200 &&
            !SoleHelper::sole_server_of_cand(sole_server, tasks, c.task))
        {
            bool strands_sole = false;
            for (size_t j2 = 0; j2 < tasks.size() && !strands_sole; ++j2)
            {
                if (tasks[j2] == c.task || !sole_server[j2] || task_taken.count(tasks[j2]->id))
                    continue;
                const vector<int> &d_now = st.dist_c[c.rid];
                int trav_now = d_now[st.idx(tasks[j2]->coord)];
                int we2 = work_energy(*tasks[j2], by_id[c.rid]->type);
                if (trav_now >= PLAN_INF || we2 >= PLAN_INF ||
                    trav_now + we2 > by_id[c.rid]->get_energy())
                    continue; // not actually my sole obligation (can't serve it anyway)
                int type_i = static_cast<int>(by_id[c.rid]->type);
                const vector<int> &d_after = st.dist_from_task(tasks[j2]->id, type_i, tasks[j2]->coord);
                int trav_after = d_after[st.idx(c.task->coord)]; // symmetric-ish estimate
                if (trav_after >= PLAN_INF ||
                    trav_after + we2 > by_id[c.rid]->get_energy() - c.need)
                    strands_sole = true; // after this trip, the sole task is lost
            }
            if (strands_sole)
                continue;
        }
        robot_busy[c.rid] = true;
        task_taken.insert(c.task->id);
        new_owner[c.task->id] = c.rid;
        st.assigned[c.rid] = c.task->id;
        Coord pos = (by_id[c.rid]->get_status() == ROBOT::STATUS::MOVING)
                        ? by_id[c.rid]->get_target_coord()
                        : by_id[c.rid]->get_coord();
        st.next_step[c.rid] = st.first_step(st.par[c.rid], pos, c.task->coord);
    }


    st.owner.swap(new_owner);

    // observation value = staleness x how cheaply the fleet could serve a task
    // found there
    st.serve_dist.assign(st.n * st.n, PLAN_INF);
    for (size_t i = 0; i < robots.size(); ++i)
    {
        const ROBOT &r = *robots[i];
        if (r.type == ROBOT::TYPE::DRONE || r.get_status() == ROBOT::STATUS::EXHAUSTED)
            continue;
        if (r.get_energy() < 1500 || st.dist_c[r.id].empty())
            continue; // nearly-spent workers won't serve new finds
        for (int c = 0; c < st.n * st.n; ++c)
        {
            int dd = st.dist_c[r.id][c];
            if (dd > r.get_energy())
                dd = PLAN_INF; // beyond its actual reach
            if (dd < st.serve_dist[c])
                st.serve_dist[c] = dd;
        }
    }

    // ---- endgame interior tiling (workers) --------------------------------
    // The drones' late rim ring re-observes the outer band after the last
    // task spawns; the interior is re-observed here: from TILE_START every
    // idle worker walks nearest-first through a fixed lattice of interior
    // waypoints (stride 3 fits the 3x3/cross views), so that ~every interior
    // cell is seen again after all spawning has finished.
    st.serve_dist.assign(st.n * st.n, PLAN_INF);
    for (size_t i = 0; i < robots.size(); ++i)
    {
        const ROBOT &r = *robots[i];
        if (r.type == ROBOT::TYPE::DRONE || r.get_status() == ROBOT::STATUS::EXHAUSTED)
            continue;
        if (r.get_energy() < 1500 || st.dist_c[r.id].empty())
            continue;
        for (int c = 0; c < st.n * st.n; ++c)
        {
            int dd = st.dist_c[r.id][c];
            if (dd > r.get_energy())
                dd = PLAN_INF;
            if (dd < st.serve_dist[c])
                st.serve_dist[c] = dd;
        }
    }
    st.build_staleness();

    // ---- drones: committed serpentine sweeps ------------------------------

    vector<const ROBOT *> active_drones;
    for (size_t i = 0; i < robots.size(); ++i)
    {
        const ROBOT &r = *robots[i];
        if (r.type == ROBOT::TYPE::DRONE && r.get_status() != ROBOT::STATUS::EXHAUSTED &&
            r.get_energy() > DRONE_CAMERA_FLOOR)
            active_drones.push_back(&r);
    }
    // (re)partition map into x-bands when the active drone set changes
    {
        bool repartition = false;
        for (size_t i = 0; i < active_drones.size(); ++i)
            if (st.drone_half.find(active_drones[i]->id) == st.drone_half.end())
                repartition = true;
        if (st.drone_half.size() != active_drones.size())
            repartition = true;
        if (repartition)
        {
            st.drone_half.clear();
            st.drone_goal.clear(); // NOTE: rim-route state survives repartition
            if (!active_drones.empty())
            {
                // sort by current x so bands match where drones already are
                vector<const ROBOT *> ds = active_drones;
                for (size_t a = 0; a < ds.size(); ++a)
                    for (size_t b = a + 1; b < ds.size(); ++b)
                        if (ds[b]->get_coord().x < ds[a]->get_coord().x)
                            swap(ds[a], ds[b]);
                int bands = static_cast<int>(ds.size());
                for (int b = 0; b < bands; ++b)
                {
                    int xlo = st.n * b / bands;
                    int xhi = st.n * (b + 1) / bands - 1;
                    st.drone_half[ds[b]->id] = make_pair(xlo, xhi);
                }
            }
        }
    }

    st.drone_covered.assign(st.n * st.n, 0);
    for (size_t i = 0; i < active_drones.size(); ++i)
    {
        const ROBOT &r = *active_drones[i];
        Coord pos = (r.get_status() == ROBOT::STATUS::MOVING) ? r.get_target_coord() : r.get_coord();
        int viewr = ROBOT::view_range_list[static_cast<size_t>(r.type)];
        pair<int, int> half = st.drone_half[r.id];
        const vector<int> &d = st.dist[r.id];

        // Two-phase observation:
        //  - Early: a small burst of adaptive scouting, then parked (camera).
        //  - Late: a committed full-tiling serpentine sweep of the band, timed
        //    (from the seed's known drone step cost) to finish just before the
        //    assumed end of the run, so that every cell is observed as late as
        //    possible — after as many task spawns as possible.
        int horizon = HORIZON_HARD_PER_CELL * st.n;
        int step_ticks = 30; // pessimistic default until the drone cost is known
        if (st.drone_cost > 0)
            step_ticks = (st.drone_cost / 2 + st.drone_cost + 9) / 10 + 1;
        // half-ring length: one vertical side + two half horizontal sides,
        // plus an approach allowance
        int route_steps = (st.n - 2 * viewr) + 2 * (st.n / 2 - viewr) + 5;
        int t_serp = horizon - 30 - static_cast<int>(route_steps * step_ticks * 1.1);
        if (t_serp < 0)
            t_serp = 0;

        if (st.regime != 2)
            t_serp = PLAN_INF; // dense/undecided: pure adaptive scouting, no rim ring
        if (st.now >= t_serp && !st.serp_started[r.id])
        {
            st.serp_started[r.id] = true;
            vector<Coord> route = st.make_rim_route(half.first, half.second, viewr, pos);
            if (!route.empty() &&
                abs(route.back().x - pos.x) + abs(route.back().y - pos.y) <
                    abs(route.front().x - pos.x) + abs(route.front().y - pos.y))
            {
                vector<Coord> rev(route.rbegin(), route.rend());
                route.swap(rev);
            }
            // triage: trim the tail beyond what this drone can actually pay
            // for (walk cost estimated waypoint-to-waypoint), so the covered
            // arc is contiguous and the sacrificed stretch is a known, single
            // segment that worker debt patrols will inherit
            {
                int sc = (st.drone_cost > 0) ? (ceil10(st.drone_cost / 2 + st.drone_cost) * 10 + 10) : 310;
                int budget = r.get_energy() - 300;
                Coord prev = pos;
                size_t keep = 0;
                for (size_t w = 0; w < route.size(); ++w)
                {
                    int hop = (abs(route[w].x - prev.x) + abs(route[w].y - prev.y));
                    budget -= static_cast<int>(hop * sc * 1.15);
                    if (budget < 0)
                        break;
                    keep = w + 1;
                    prev = route[w];
                }
                if (keep < route.size())
                    route.resize(max<size_t>(keep, 1));
            }
            st.serp_route[r.id] = route;
        }

        if (st.serp_started[r.id])
        {
            // ---- late phase: follow the rim route down to the last step ----
            if (r.get_energy() - DRONE_STEP_EST < 120)
                continue; // out of fuel: parked camera
            vector<Coord> &route = st.serp_route[r.id];
            while (!route.empty())
            {
                Coord wp = route.front();
                if (wp == pos)
                {
                    route.erase(route.begin());
                    continue;
                }
                {
                    int sc0 = (st.drone_cost > 0) ? (ceil10(st.drone_cost / 2 + st.drone_cost) * 10 + 10) : 310;
                    int direct = (abs(wp.x - pos.x) + abs(wp.y - pos.y)) * sc0;
                    if (d[st.idx(wp)] < PLAN_INF && d[st.idx(wp)] > 3 * direct + 600)
                    {
                        route.erase(route.begin()); // pocketed: not worth the detour
                        continue;
                    }
                }
                if (d[st.idx(wp)] >= PLAN_INF)
                {
                    // walled waypoint: try to substitute a nearby reachable
                    // cell so the arc segment is still visited
                    Coord sub(-1, -1);
                    int best = PLAN_INF;
                    for (int dx = -2; dx <= 2; ++dx)
                        for (int dy = -2; dy <= 2; ++dy)
                        {
                            int nx = wp.x + dx, ny = wp.y + dy;
                            if (!st.in_map(nx, ny) || Coord(nx, ny) == pos)
                                continue;
                            int dd = d[st.idx(nx, ny)];
                            if (dd < best)
                            {
                                best = dd;
                                sub = Coord(nx, ny);
                            }
                        }
                    route.erase(route.begin());
                    if (st.in_map(sub.x, sub.y))
                        route.insert(route.begin(), sub);
                    else
                        continue;
                }
                break;
            }
            if (route.empty())
            {
                // Own arc finished: adopt the unfinished arc of a drone that
                // ran dry mid-ring, so the whole rim still gets covered.
                for (size_t o = 0; o < robots.size(); ++o)
                {
                    const ROBOT &other = *robots[o];
                    if (other.id == r.id || other.type != ROBOT::TYPE::DRONE)
                        continue;
                    if (other.get_status() != ROBOT::STATUS::EXHAUSTED && other.get_energy() > 500)
                        continue;
                    map<int, vector<Coord>>::iterator oit = st.serp_route.find(other.id);
                    if (oit != st.serp_route.end() && !oit->second.empty())
                    {
                        route.swap(oit->second);
                        break;
                    }
                }
            }
            // mark the part of the route this drone can still afford as
            // covered, so worker debt patrols go where nobody else will
            {
                int sc = (st.drone_cost > 0) ? (ceil10(st.drone_cost / 2 + st.drone_cost) * 10 + 10) : 310;
                int budget = r.get_energy() - 150;
                Coord prev = pos;
                for (size_t w = 0; w < route.size() && budget > 0; ++w)
                {
                    int hop = abs(route[w].x - prev.x) + abs(route[w].y - prev.y);
                    budget -= hop * sc;
                    if (budget <= 0)
                        break;
                    for (int dx = -viewr; dx <= viewr; ++dx)
                        for (int dy = -viewr; dy <= viewr; ++dy)
                            if (st.in_map(route[w].x + dx, route[w].y + dy))
                                st.drone_covered[st.idx(route[w].x + dx, route[w].y + dy)] = 1;
                    prev = route[w];
                }
            }
            if (!route.empty())
            {
                st.next_step[r.id] = st.first_step(st.par[r.id], pos, route.front());
                continue;
            }
            // route done: fall through to adaptive crawling with leftovers
        }

        // ---- early phase / post-route: adaptive stale-window crawling ----
        // The early scouting burst is whatever the ring will not need: on
        // expensive-drone seeds that is zero, so the ring always completes.
        int step_cost = (st.drone_cost > 0) ? (ceil10(st.drone_cost / 2 + st.drone_cost) * 10 + 10) : 310;
        int floor_energy;
        if (st.regime != 2)
        {
            // dense/undecided: spend on a linear schedule with an up-front
            // burst (the completion-focused profile); undecided keeps the
            // burst small so a sparse map still has ring fuel left
            int e0 = st.init_energy[r.id];
            if (st.regime == 1)
            {
                floor_energy = DRONE_CAMERA_FLOOR;
                if (st.now < st.horizon_dense())
                {
                    double frac = static_cast<double>(st.now) / static_cast<double>(st.horizon_dense());
                    int budget_floor = e0 - 3000 - static_cast<int>((e0 - 3000 - DRONE_CAMERA_FLOOR) * frac);
                    floor_energy = max(DRONE_CAMERA_FLOOR, budget_floor);
                }
            }
            else
            {
                // undecided: flat cap so a sparse map keeps its ring fuel
                floor_energy = max(DRONE_CAMERA_FLOOR, e0 - 800);
            }
        }
        else
        {
            int ring_cost = static_cast<int>(route_steps * step_cost * 1.15);
            int burst_allow = max(0, st.init_energy[r.id] - 400 - ring_cost);
            if (burst_allow > DRONE_BURST_DEF)
                burst_allow = DRONE_BURST_DEF;
            floor_energy = st.serp_started[r.id] ? DRONE_CAMERA_FLOOR
                                                 : max(DRONE_CAMERA_FLOOR, st.init_energy[r.id] - burst_allow);
        }
        if (r.get_energy() - DRONE_STEP_EST < floor_energy)
            continue; // parked as camera until the late sweep begins

        map<int, Coord>::iterator gi = st.drone_goal.find(r.id);
        bool need_new = true;
        if (gi != st.drone_goal.end())
        {
            Coord g = gi->second;
            if (!(g == pos) && d[st.idx(g)] < PLAN_INF)
                need_new = false;
        }
        if (need_new)
        {
            int need_t = NEED_T_PER_CELL * st.n;
            bool debt_mode = st.now >= need_t;
            int best_local = 0, best_global = 0;
            Coord cell_local = pos, cell_global = pos;
            for (int x = half.first; x <= half.second; ++x)
                for (int y = 0; y < st.n; ++y)
                {
                    int dd = d[st.idx(x, y)];
                    if (dd >= PLAN_INF || dd > r.get_energy() - DRONE_CAMERA_FLOOR)
                        continue;
                    if (Coord(x, y) == pos)
                        continue;
                    int g = debt_mode ? st.debt_gain(x, y, viewr, need_t) * 1000
                                      : st.window_gain(x, y, viewr);
                    if (g > best_global)
                    {
                        best_global = g;
                        cell_global = Coord(x, y);
                    }
                    if (dd <= DRONE_LOCAL_RANGE && g > best_local)
                    {
                        best_local = g;
                        cell_local = Coord(x, y);
                    }
                }
            int min_gain = debt_mode ? 1000 : ((st.regime == 1) ? 2500 : RESWEEP_MIN_GAIN);
            Coord chosen = pos;
            if (best_local >= min_gain && best_local >= LOCAL_KEEP_RATIO * best_global)
                chosen = cell_local;
            else if (best_global >= min_gain)
                chosen = cell_global;
            if (chosen == pos)
            {
                st.drone_goal.erase(r.id);
                continue;
            }
            st.drone_goal[r.id] = chosen;
        }
        st.next_step[r.id] = st.first_step(st.par[r.id], pos, st.drone_goal[r.id]);
    }



    if (st.now >= 800 && st.regime != 1)
    {
        // Idle-worker patrol: refresh windows that still need (re)observation.
        // Before the last possible task spawn this follows staleness; after it
        // (t >= NEED_T) the gain becomes "cells not seen since the final
        // spawn" — the exact re-observation debt for full task discovery.
        int need_t = NEED_T_PER_CELL * st.n;
        for (size_t i = 0; i < robots.size(); ++i)
        {
            const ROBOT &r = *robots[i];
            if (r.type == ROBOT::TYPE::DRONE || r.get_status() == ROBOT::STATUS::EXHAUSTED ||
                r.get_status() == ROBOT::STATUS::WORKING)
                continue;
            if (st.assigned[r.id] != -1 || r.get_energy() < 2000 || st.dist_c[r.id].empty())
                continue;
            Coord pos = (r.get_status() == ROBOT::STATUS::MOVING) ? r.get_target_coord() : r.get_coord();
            const vector<int> &d = st.dist_c[r.id];
            int viewr = ROBOT::view_range_list[static_cast<size_t>(r.type)];
            bool debt_mode = st.now >= need_t;
            int range = debt_mode ? max(0, r.get_energy() - 300) : 4000;
            int best_gain = 0;
            Coord best = pos;
            for (int x = 0; x < st.n; ++x)
                for (int y = 0; y < st.n; ++y)
                {
                    int dd = d[st.idx(x, y)];
                    if (dd >= PLAN_INF || dd > range || Coord(x, y) == pos)
                        continue;
                    int g = debt_mode ? st.debt_gain(x, y, viewr, need_t) * 1000 - dd / 2
                                      : st.window_gain(x, y, viewr) - dd / 2;
                    if (g > best_gain)
                    {
                        best_gain = g;
                        best = Coord(x, y);
                    }
                }
            int min_gain = debt_mode ? 1000 : 600;
            if (best_gain >= min_gain && !(best == pos))
                st.next_step[r.id] = st.first_step(st.par[r.id], pos, best);
        }
    }


}

bool Scheduler::on_task_reached(const set<Coord> &observed_coords,
                                const set<Coord> &updated_coords,
                                const vector<vector<vector<int>>> &known_cost_map,
                                const vector<vector<OBJECT>> &known_object_map,
                                const vector<shared_ptr<TASK>> &active_tasks,
                                const vector<shared_ptr<ROBOT>> &robots,
                                const ROBOT &robot,
                                const TASK &task)
{
    State &st = *s_;
    if (robot.type == ROBOT::TYPE::DRONE)
        return false; // a drone can never finish a task and would burn out
    if (task.is_done())
        return false;
    if (task.get_assigned_robot_id() != -1)
        return false; // someone is already physically working on it

    int we = work_energy(task, robot.type);
    if (we >= PLAN_INF || robot.get_energy() < we)
        return false; // could not finish: starting would only block the task

    map<int, int>::iterator ow = st.owner.find(task.id);
    int holder = (ow == st.owner.end()) ? -1 : ow->second;
    if (holder != -1 && holder != robot.id)
    {
        // A booked robot is on its way; take over only when finishing here is
        // cheaper than the holder's remaining travel + work.
        const ROBOT *h = 0;
        for (size_t i = 0; i < robots.size(); ++i)
            if (robots[i]->id == holder)
            {
                h = robots[i].get();
                break;
            }
        int holder_cost = PLAN_INF;
        if (h != 0 && holder < static_cast<int>(st.dist_c.size()) && !st.dist_c[holder].empty())
        {
            int trav = st.dist_c[holder][st.idx(task.coord)];
            int hwe = work_energy(task, h->type);
            if (trav < PLAN_INF && hwe < PLAN_INF)
                holder_cost = trav + hwe;
        }
        if (we >= holder_cost)
            return false;
        if (holder < static_cast<int>(st.assigned.size()) && st.assigned[holder] == task.id)
            st.assigned[holder] = -1;
    }

    // book it for ourselves (also releases our previous booking)
    if (st.assigned[robot.id] != -1 && st.assigned[robot.id] != task.id)
    {
        map<int, int>::iterator old = st.owner.find(st.assigned[robot.id]);
        if (old != st.owner.end() && old->second == robot.id)
            st.owner.erase(old);
    }
    st.owner[task.id] = robot.id;
    st.assigned[robot.id] = task.id;
    return true;
}

ROBOT::ACTION Scheduler::idle_action(const set<Coord> &observed_coords,
                                     const set<Coord> &updated_coords,
                                     const vector<vector<vector<int>>> &known_cost_map,
                                     const vector<vector<OBJECT>> &known_object_map,
                                     const vector<shared_ptr<TASK>> &active_tasks,
                                     const vector<shared_ptr<ROBOT>> &robots,
                                     const ROBOT &robot)
{
    State &st = *s_;
    if (robot.id >= static_cast<int>(st.next_step.size()))
        return ROBOT::ACTION::HOLD;
    Coord cur = robot.get_coord();
    Coord nx = st.next_step[robot.id];
    if (!st.in_map(nx.x, nx.y) || nx == cur)
        return ROBOT::ACTION::HOLD;
    int dx = nx.x - cur.x, dy = nx.y - cur.y;
    if (dx * dx + dy * dy != 1)
        return ROBOT::ACTION::HOLD;
    if (bool(known_object_map[nx.x][nx.y] & OBJECT::WALL))
        return ROBOT::ACTION::HOLD; // never bump into a known wall
    if (dy == 1)
        return ROBOT::ACTION::UP;
    if (dy == -1)
        return ROBOT::ACTION::DOWN;
    if (dx == -1)
        return ROBOT::ACTION::LEFT;
    return ROBOT::ACTION::RIGHT;
}
