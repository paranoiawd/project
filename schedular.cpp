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
//  * The budget is the whole problem.  A robot carries 6 energy per tick of the
//    run, so it can act for at most 60% of it; the four workers together can
//    pay for roughly sixteen average task trips and the two drones for roughly
//    one and a half sweeps of the map.  Nothing here can be maximised
//    independently -- observation is paid for out of the same pool that
//    completes tasks.
//
//  * Observation is therefore valued in the currency of completions.  A cell is
//    worth looking at in proportion to
//        (expected undiscovered tasks sitting on it)
//      x (probability the fleet could still serve a task found there).
//    The first factor follows the dispatcher's arrival ramp: a cell nobody has
//    ever seen may hold one of the initial batch, and a cell seen at tick t can
//    only have gained the spawns released since t -- so re-observation is worth
//    nothing before the first spawn.  The second factor folds in the cheapest
//    worker's travel energy *and* the ticks left in the run, which is what
//    stops the fleet buying discoveries it can no longer convert.  Scouts pick
//    targets by value per unit of energy, never by raw value.
//
//  * Workers (caterpillar/wheel -- drones can never finish a task) are matched
//    to tasks by greedy assignment on Dijkstra travel + work energy over the
//    known cost map, biased by a cheapest-insertion tour so the fleet follows
//    routes instead of per-tick zigzags, with a waiting clock that stops far
//    tasks from being chased before a cheaper server has had a chance to free
//    up.  A worker with nothing it wants to serve patrols instead of holding:
//    energy still in the tank when the run ends is a pure loss.
//
//  * Drones are pure scouts, one x-band each, spending on a paced line so that
//    coverage is spread across the run rather than front-loaded; out-of-budget
//    drones park and keep observing as fixed cameras.
//
//  * on_task_reached only accepts a task when the robot type can complete it,
//    nobody else is physically working on it, and the robot has enough energy
//    to finish; a robot standing on a task may also opportunistically take it
//    over when that is cheaper than the booked robot's remaining travel+work.
//
//  * Everything is replanned from scratch every tick; no state survives that
//    the simulator could contradict.
// ---------------------------------------------------------------------------

namespace
{
    // ---- tuning knobs -----------------------------------------------------
    const double UNKNOWN_PEN[3] = {1.05, 1.15, 1.15}; // pathfinding pessimism on unknown cells
    const int EST_CELL_COST[3] = {160, 299, 448};     // expected cell cost per type (drone/cat/wheel)
    double ASSIGN_STICKY = 0.85;                // cost discount for keeping an assignment
    const int WORKER_MIN_ENERGY = 30;                 // below this a worker is effectively retired
    int WORKER_TRAVEL_CAP = 3800;               // normal per-assignment travel budget
    const int DRONE_CAMERA_FLOOR = 400;               // drones never spend below this (parked sensor)
    const int DRONE_STEP_EST = 300;                   // rough energy for one drone step
    // Observation-value and patrol knobs.  These are readable from the
    // environment (SCHED_T_*) so the bench harness can sweep them without a
    // rebuild; every default below is the value the reported numbers were
    // measured on, and an unset environment reproduces them exactly.
    double SERVE_W_HI = 3.0;    // weight at zero serve cost
    double SERVE_W_LO = 0.3;    // weight for unreachable cells
    double SERVE_W_SLOPE = 5000.0; // energy per unit of weight lost
    int SERVE_RAMP = 260;       // ticks of slack over which value ramps to zero
    int SCOUT_K = 500;          // energy offset in the value/cost ratio (~one step)
    int SCOUT_MIN_RATIO = 900;  // minimum value per unit cost for a drone to move
    int PATROL_MIN_ENERGY = 1400;// a worker below this keeps its energy for tasks
    int PATROL_MIN_RATIO = 1400;// workers see fewer cells per step than drones
    int PATROL_START = 800;     // first tick idle workers may patrol
    int PATROL_DISPERSE = 1200; // x1000 weight pushing patrols away from other workers
    int PATROL_LATE_T = 1200;   // after this tick there is nothing left to save for
    int PATROL_LATE_ENERGY = 400;// so the patrol floor drops to here
    int DRONE_PACE_T = 1700;    // ticks over which a drone's fuel is spread (0 = no pacing)
    int DRONE_BURST = 3000;     // fuel a drone may spend ahead of that line
    double SERVE_W_DEAD = 0.08; // residual weight once nothing found here could
                                // still be served: raw discovery is still worth
                                // something to a robot whose fuel is otherwise lost

    inline double envd(const char *k, double d)
    {
        const char *v = getenv(k);
        return v ? atof(v) : d;
    }
    inline int envi(const char *k, int d)
    {
        const char *v = getenv(k);
        return v ? atoi(v) : d;
    }
    int STARVE_AGE = 250;                       // discovered-but-unserved ticks before the
                                                      // travel cap is waived for a task
    int TASK_MAGNET = 350;                      // path bonus for stepping onto a free task
    const int LOCK_DIST = 700;                        // don't rebook a nearly-reached assignment
    const int HORIZON_HARD_PER_CELL = 100;            // assumed hard end ~ 100 * map_size ticks
    int ENDGAME_TICKS_DEF = 600;                // endgame window before the hard horizon
    double TOUR_FIRST_BONUS = 0.55;             // score factor for a tour's first leg
    const int TOUR_MAX_LEN = 8;                       // max tasks per planned tour
    const int PLAN_INF = 1000000000;

    const int DXS[4] = {0, 0, -1, 1};
    const int DYS[4] = {1, -1, 0, 0};

    inline int ceil10(int v) { return (v + 9) / 10; }

    void load_tunables()
    {
        static bool done = false;
        if (done)
            return;
        done = true;
        SERVE_W_HI = envd("SCHED_T_SWHI", SERVE_W_HI);
        SERVE_W_LO = envd("SCHED_T_SWLO", SERVE_W_LO);
        SERVE_W_SLOPE = envd("SCHED_T_SWSL", SERVE_W_SLOPE);
        SERVE_RAMP = envi("SCHED_T_SRAMP", SERVE_RAMP);
        SCOUT_K = envi("SCHED_T_SK", SCOUT_K);
        SCOUT_MIN_RATIO = envi("SCHED_T_SMR", SCOUT_MIN_RATIO);
        PATROL_MIN_ENERGY = envi("SCHED_T_PME", PATROL_MIN_ENERGY);
        PATROL_MIN_RATIO = envi("SCHED_T_PMR", PATROL_MIN_RATIO);
        PATROL_START = envi("SCHED_T_PSTART", PATROL_START);
        PATROL_DISPERSE = envi("SCHED_T_PDISP", PATROL_DISPERSE);
        PATROL_LATE_T = envi("SCHED_T_PLATE", PATROL_LATE_T);
        PATROL_LATE_ENERGY = envi("SCHED_T_PMEL", PATROL_LATE_ENERGY);
        WORKER_TRAVEL_CAP = envi("SCHED_T_WTC", WORKER_TRAVEL_CAP);
        STARVE_AGE = envi("SCHED_T_STARVE", STARVE_AGE);
        ENDGAME_TICKS_DEF = envi("SCHED_T_ENDG", ENDGAME_TICKS_DEF);
        TASK_MAGNET = envi("SCHED_T_MAGNET", TASK_MAGNET);
        DRONE_PACE_T = envi("SCHED_T_DPACE", DRONE_PACE_T);
        DRONE_BURST = envi("SCHED_T_DBURST", DRONE_BURST);
        SERVE_W_DEAD = envd("SCHED_T_SWDEAD", SERVE_W_DEAD);
        TOUR_FIRST_BONUS = envd("SCHED_T_TFB", TOUR_FIRST_BONUS);
        ASSIGN_STICKY = envd("SCHED_T_STICKY", ASSIGN_STICKY);
    }

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

    // drone sweep state
    map<int, pair<int, int>> drone_half; // drone id -> [x_lo, x_hi] band
    map<int, Coord> drone_goal;          // drone id -> committed sweep target.
                                         // Never re-evaluated before arrival: a
                                         // drone that re-picks every tick
                                         // oscillates, because approaching a
                                         // target observes away its own value.


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

    // ---- observation value -------------------------------------------------
    // serve_dist[cell] = min over alive workers of clean travel energy; used to
    // weight observation value: a task found near a worker is cheap to serve,
    // one found in a far corner would mostly go unserved anyway.
    vector<int> serve_dist;

    int horizon() const { return HORIZON_HARD_PER_CELL * n; }

    // Fraction (x1000) of the "late" spawn batch that has arrived by tick t.
    // The dispatcher releases the second half of the tasks between horizon/4
    // and ~0.7*horizon at a constant cadence, so the arrival curve is a ramp.
    int spawned_frac(int t) const
    {
        int lo = horizon() / 4, hi = horizon() * 11 / 16;
        if (t <= lo)
            return 0;
        if (t >= hi)
            return 1000;
        return (t - lo) * 1000 / (hi - lo);
    }

    // Value of observing each cell, in units of "expected completions x1e-?".
    // Two independent factors:
    //   mass  = expected number of *undiscovered* tasks sitting on the cell.
    //           A never-seen cell may hold one of the initial batch (1000) plus
    //           any late spawn so far; a cell seen at tick ls can only have
    //           gained the spawns released since then.
    //   serve = probability the fleet could actually complete a task found
    //           here: it must be reachable with the energy a worker has left
    //           *and* within the ticks remaining.  This is what makes late
    //           observation of far cells worthless, and it is the difference
    //           between raw discovery and discovery that pays.
    void build_staleness()
    {
        stale.assign(n * n, 0);
        int ft = spawned_frac(now);
        int left = horizon() - now;
        for (int x = 0; x < n; ++x)
            for (int y = 0; y < n; ++y)
            {
                if ((*obj_map)[x][y] == OBJECT::WALL)
                    continue;
                int ls = last_seen[x][y];
                int mass = (ls < 0) ? (1000 + ft) : (ft - spawned_frac(ls));
                if (mass <= 0)
                    continue;
                double w = 1.0;
                if (!serve_dist.empty())
                {
                    int sd = serve_dist[idx(x, y)];
                    if (sd >= PLAN_INF)
                        w = SERVE_W_LO;
                    else
                    {
                        w = SERVE_W_HI - static_cast<double>(sd) / SERVE_W_SLOPE;
                        if (w < SERVE_W_LO)
                            w = SERVE_W_LO;
                        if (w > SERVE_W_HI)
                            w = SERVE_W_HI;
                        // a find nobody can still walk to before the run ends is
                        // worth nothing, and the value ramps down as that
                        // deadline approaches
                        int slack = left - (sd + 100) / 10;
                        if (slack <= 0)
                            w = SERVE_W_DEAD;
                        else if (slack < SERVE_RAMP)
                        {
                            double f = static_cast<double>(slack) / SERVE_RAMP;
                            w = w * f + SERVE_W_DEAD * (1.0 - f);
                        }
                    }
                }
                stale[idx(x, y)] = static_cast<int>(mass * w);
            }
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
    load_tunables();
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

    // serve_dist[c] = cheapest travel energy any worker still able to act could
    // pay to reach c; it is what turns raw observation value into value that
    // can actually become a completion.
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
            st.drone_goal.clear();
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

    for (size_t i = 0; i < active_drones.size(); ++i)
    {
        const ROBOT &r = *active_drones[i];
        Coord pos = (r.get_status() == ROBOT::STATUS::MOVING) ? r.get_target_coord() : r.get_coord();
        int viewr = ROBOT::view_range_list[static_cast<size_t>(r.type)];
        pair<int, int> half = st.drone_half[r.id];
        const vector<int> &d = st.dist[r.id];

        // Pacing.  Spread the drone's fuel over DRONE_PACE_T ticks after an
        // up-front burst: pacing trades earlier coverage (worth more, because an
        // early find can still be served) against later coverage (worth more,
        // because it catches spawns that have happened since).  Measured on 300
        // seeds, moving the line to 1200 buys +0.13 completed for -0.57
        // discovered and 2000 buys the reverse, so 1700 is the joint optimum.
        // A drone below the line parks and keeps observing as a fixed camera.
        int floor_energy = DRONE_CAMERA_FLOOR;
        if (DRONE_PACE_T > 0 && st.now < DRONE_PACE_T)
        {
            int e0 = st.init_energy[r.id];
            double frac = static_cast<double>(st.now) / static_cast<double>(DRONE_PACE_T);
            int budget_floor = e0 - DRONE_BURST -
                               static_cast<int>((e0 - DRONE_BURST - DRONE_CAMERA_FLOOR) * frac);
            floor_energy = max(DRONE_CAMERA_FLOOR, budget_floor);
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
            // Value per unit of energy, not raw value: walking half the map to
            // refresh one window costs more than the find is worth.  The +K
            // offset keeps an adjacent cell from winning on a rounding artefact.
            int best_ratio = 0;
            Coord chosen = pos;
            for (int x = half.first; x <= half.second; ++x)
                for (int y = 0; y < st.n; ++y)
                {
                    int dd = d[st.idx(x, y)];
                    if (dd >= PLAN_INF || dd > r.get_energy() - DRONE_CAMERA_FLOOR)
                        continue;
                    if (Coord(x, y) == pos)
                        continue;
                    int g = st.window_gain(x, y, viewr);
                    if (g <= 0)
                        continue;
                    int ratio = static_cast<int>(static_cast<long long>(g) * 1000 / (dd + SCOUT_K));
                    if (ratio > best_ratio)
                    {
                        best_ratio = ratio;
                        chosen = Coord(x, y);
                    }
                }
            if (best_ratio < SCOUT_MIN_RATIO || chosen == pos)
            {
                st.drone_goal.erase(r.id);
                continue;
            }
            st.drone_goal[r.id] = chosen;
        }
        st.next_step[r.id] = st.first_step(st.par[r.id], pos, st.drone_goal[r.id]);
    }



    if (st.now >= PATROL_START)
    {
        // Idle-worker patrol.  A worker with no task it wants to serve has
        // nothing better to do with its energy than look for one — leftover
        // energy at the end of the run is a pure loss.  The target is chosen by
        // value per unit of energy for the same reason as the drones': the old
        // argmax-gain rule happily walked 4000 energy to refresh one 3x3 window.
        for (size_t i = 0; i < robots.size(); ++i)
        {
            const ROBOT &r = *robots[i];
            if (r.type == ROBOT::TYPE::DRONE || r.get_status() == ROBOT::STATUS::EXHAUSTED ||
                r.get_status() == ROBOT::STATUS::WORKING)
                continue;
            // Energy held back for a task that may still turn up is worth
            // nothing once no such task could still be served: past
            // PATROL_LATE_T the reserve is released to observation.
            int floor_e = (st.now >= PATROL_LATE_T) ? PATROL_LATE_ENERGY : PATROL_MIN_ENERGY;
            if (st.assigned[r.id] != -1 || r.get_energy() < floor_e || st.dist_c[r.id].empty())
                continue;
            Coord pos = (r.get_status() == ROBOT::STATUS::MOVING) ? r.get_target_coord() : r.get_coord();
            const vector<int> &d = st.dist_c[r.id];
            int viewr = ROBOT::view_range_list[static_cast<size_t>(r.type)];
            int range = max(0, r.get_energy() - floor_e);
            int best_ratio = 0;
            Coord best = pos;
            for (int x = 0; x < st.n; ++x)
                for (int y = 0; y < st.n; ++y)
                {
                    int dd = d[st.idx(x, y)];
                    if (dd >= PLAN_INF || dd > range || Coord(x, y) == pos)
                        continue;
                    int g = st.window_gain(x, y, viewr);
                    if (g <= 0)
                        continue;
                    if (PATROL_DISPERSE > 0)
                    {
                        // A patrol is also a repositioning: standing where no
                        // other worker can cheaply reach shortens the trip to
                        // whatever spawns there next.  Free to steer, since the
                        // move is happening anyway.
                        int far = PLAN_INF;
                        for (size_t k = 0; k < robots.size(); ++k)
                        {
                            const ROBOT &o = *robots[k];
                            if (o.id == r.id || o.type == ROBOT::TYPE::DRONE ||
                                o.get_status() == ROBOT::STATUS::EXHAUSTED ||
                                st.dist_c[o.id].empty())
                                continue;
                            far = min(far, st.dist_c[o.id][st.idx(x, y)]);
                        }
                        if (far < PLAN_INF)
                        {
                            int b = min(far, 5000) * PATROL_DISPERSE / 5000;
                            g = static_cast<int>(static_cast<long long>(g) * (1000 + b) / 1000);
                        }
                    }
                    int ratio = static_cast<int>(static_cast<long long>(g) * 1000 / (dd + SCOUT_K));
                    if (ratio > best_ratio)
                    {
                        best_ratio = ratio;
                        best = Coord(x, y);
                    }
                }
            if (best_ratio >= PATROL_MIN_RATIO && !(best == pos))
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
