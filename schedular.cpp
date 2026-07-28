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
    int WORKER_TRAVEL_CAP = 0;               // normal per-assignment travel budget
    int DRONE_CAMERA_FLOOR = 100;                     // drones never spend below this (parked sensor)
    int DRONE_STEP_EST = 120;                         // rough energy for one drone step
    int DRONE_TAIL_FLOOR = 10;                        // reserve once no spawns remain
    // Observation-value and patrol knobs.  These are readable from the
    // environment (SCHED_T_*) so the bench harness can sweep them without a
    // rebuild; every default below is the value the reported numbers were
    // measured on, and an unset environment reproduces them exactly.
    // Serve weighting: how much more an observation is worth where the fleet
    // could actually act on a find.  It is NEUTRAL by default (all three
    // weights 1.0) and that is a measured result, not an oversight.  It existed
    // to stop a scout walking half the map to refresh one far window -- but
    // that was an artefact of scoring a target by the window it ends on.  Once
    // a trip is scored by everything it sees on the way (SCOUT_PATHVAL), the
    // energy denominator already prices distance, and weighting it a second
    // time suppressed exactly the sweeps that pay.  Flattening it on top of the
    // OLD value model costs 0.07 completed; on top of the new one it is worth
    // +0.23 completed and +0.39 discovered (500 fresh seeds, paired).
    // The knobs are kept so the trade-off can be re-swept.
    double SERVE_W_HI = 1.0;    // weight at zero serve cost
    double SERVE_W_LO = 1.0;    // weight for unreachable cells
    double SERVE_W_SLOPE = 5000.0; // energy per unit of weight lost
    int SERVE_RAMP = 260;       // ticks of slack over which value ramps to zero
    int SCOUT_K = 500;          // energy offset in the value/cost ratio (~one step)
    int SCOUT_MIN_RATIO = 900;  // minimum value per unit cost for a drone to move
    int SCOUT_PATHVAL = 1;      // score a scout target by everything its path sees,
                                // not just the window it ends on (0 = old rule)
    int PATH_TIEBREAK = 1;      // among equally cheap paths, take the one that
                                // observes the most (costs no energy at all)
    int SCOUT_RECOMMIT = 0;     // % margin at which a drone abandons a committed
                                // target (0 = hold it until reached)
    int PATROL_MIN_ENERGY = 1400;// a worker below this keeps its energy for tasks
    int PATROL_MIN_RATIO = 300; // workers see fewer cells per step than drones
    int PATROL_DISPERSE = 1200; // x1000 weight pushing patrols away from other workers
    int PLAN_SLACK = 0;   // ticks of margin a route must finish inside
    int PLAN_ITERS = 12;        // local-search rounds per tick
    int EXACT_MAX = 16;         // solve the fleet plan exactly when at most this many
                                // free tasks are known (0 = always use local search)
    int PATROL_LATE_T = 1200;   // after this tick there is nothing left to save for
    int PATROL_LATE_ENERGY = 50; // so the patrol floor drops to here
    int DRONE_PACE_T = 1900;    // ticks over which a drone's fuel is spread (0 = no pacing).
                                // 1700 was the joint optimum for the old value
                                // model; a sweep that is scored honestly pays to
                                // run later (1800 gives back 0.13 discovered,
                                // 2000 costs 0.08 completed).
    int DRONE_BURST = 3000;     // fuel a drone may spend ahead of that line
    double SERVE_W_DEAD = 1.0;  // residual weight once nothing found here could
                                // still be served: raw discovery is still worth
                                // something to a robot whose fuel is otherwise lost
                                // (== SERVE_W_HI, so the ramp is neutral too)

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
    int STARVE_AGE = 800;                       // discovered-but-unserved ticks before the
                                                      // travel cap is waived for a task
    int TASK_MAGNET = 350;                      // path bonus for stepping onto a free task
    const int LOCK_DIST = 700;                        // don't rebook a nearly-reached assignment
    const int HORIZON_HARD_PER_CELL = 100;            // assumed hard end ~ 100 * map_size ticks
    int ENDGAME_TICKS_DEF = 1200;                // endgame window before the hard horizon
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
        SCOUT_PATHVAL = envi("SCHED_T_SPV", SCOUT_PATHVAL);
        PATH_TIEBREAK = envi("SCHED_T_PTB", PATH_TIEBREAK);
        SCOUT_RECOMMIT = envi("SCHED_T_SRC", SCOUT_RECOMMIT);
        PATROL_MIN_ENERGY = envi("SCHED_T_PME", PATROL_MIN_ENERGY);
        PATROL_MIN_RATIO = envi("SCHED_T_PMR", PATROL_MIN_RATIO);
        PLAN_SLACK = envi("SCHED_T_SLACK", PLAN_SLACK);
        DRONE_CAMERA_FLOOR = envi("SCHED_T_DFLOOR", DRONE_CAMERA_FLOOR);
        DRONE_STEP_EST = envi("SCHED_T_DSTEP", DRONE_STEP_EST);
        DRONE_TAIL_FLOOR = envi("SCHED_T_DTAIL", DRONE_TAIL_FLOOR);
        PATROL_DISPERSE = envi("SCHED_T_PDISP", PATROL_DISPERSE);
        PLAN_ITERS = envi("SCHED_T_PITER", PLAN_ITERS);
        EXACT_MAX = envi("SCHED_T_EXACT", EXACT_MAX);
        PATROL_LATE_T = envi("SCHED_T_PLATE", PATROL_LATE_T);
        PATROL_LATE_ENERGY = envi("SCHED_T_PMEL", PATROL_LATE_ENERGY);
        WORKER_TRAVEL_CAP = envi("SCHED_T_WTC", WORKER_TRAVEL_CAP);
        STARVE_AGE = envi("SCHED_T_STARVE", STARVE_AGE);
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
    map<int, vector<int>> prev_route; // robot id -> planned task ids, last tick
    map<int, int> work_since;         // robot id -> tick it started its current job

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

    // `tb`, when given, is a direction-indexed grid of observation value (see
    // build_edge_val).  It never changes which cells are cheapest to reach --
    // it only decides *which* of the equally cheap paths is taken, so steering
    // by it costs exactly zero energy.  Shortest paths in this grid are rarely
    // unique, and the tie is otherwise settled by whichever order the queue
    // happened to pop, which is worth nothing at all.
    void dijkstra(const Coord &src, int type, vector<int> &d, vector<int> &p,
                  bool use_magnet = true, const vector<int> *tb = 0) const
    {
        d.assign(n * n, PLAN_INF);
        p.assign(n * n, -1);
        vector<int> pv;
        if (tb)
            pv.assign(n * n, 0); // observation value collected along the path
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
                    if (tb)
                        pv[v] = pv[top.second] + (*tb)[k * n * n + v];
                    pq.push(QE(nd, v));
                }
                else if (tb && nd == d[v])
                {
                    // same energy, so this is free: keep the better-observing one.
                    // v cannot have been popped yet (nd > d[u] >= every popped
                    // key), so nothing downstream has been settled from it.
                    int cand = pv[top.second] + (*tb)[k * n * n + v];
                    if (cand > pv[v])
                    {
                        pv[v] = cand;
                        p[v] = top.second;
                    }
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

    // Value of the cells a robot standing at (cx,cy) would actually see.  The
    // view shape matters: a wheel sees a cross of 5 cells, not the 9 of the
    // square its radius suggests, and counting the square made wheel patrols
    // look ~80% more productive than they are.
    int window_gain(int cx, int cy, int r, bool cross) const
    {
        int g = 0;
        if (cross)
        {
            for (int xx = max(cx - r, 0); xx <= min(cx + r, n - 1); ++xx)
                g += stale[idx(xx, cy)];
            for (int yy = max(cy - r, 0); yy <= min(cy + r, n - 1); ++yy)
                if (yy != cy)
                    g += stale[idx(cx, yy)];
            return g;
        }
        for (int xx = max(cx - r, 0); xx <= min(cx + r, n - 1); ++xx)
            for (int yy = max(cy - r, 0); yy <= min(cy + r, n - 1); ++yy)
                g += stale[idx(xx, yy)];
        return g;
    }

    // Is (px,py) inside the view of a robot standing at (cx,cy)?
    bool in_view(int cx, int cy, int px, int py, int r, bool cross) const
    {
        int ax = px - cx, ay = py - cy;
        if (ax < 0)
            ax = -ax;
        if (ay < 0)
            ay = -ay;
        if (cross)
            return (ay == 0 && ax <= r) || (ax == 0 && ay <= r);
        return ax <= r && ay <= r;
    }

    // Value of the cells that come into view for the first time when stepping
    // from u to v.  For a 5x5 view moving one cell this is the leading column
    // of five -- the other twenty were already visible from u and re-seeing
    // them buys nothing.
    int step_gain(int ux, int uy, int vx, int vy, int r, bool cross) const
    {
        int g = 0;
        for (int xx = max(vx - r, 0); xx <= min(vx + r, n - 1); ++xx)
            for (int yy = max(vy - r, 0); yy <= min(vy + r, n - 1); ++yy)
            {
                if (cross && !in_view(vx, vy, xx, yy, r, true))
                    continue;
                if (in_view(ux, uy, xx, yy, r, cross))
                    continue;
                g += stale[idx(xx, yy)];
            }
        return g;
    }

    // pg[v] = the observation value a walk from src to v collects *on the way*,
    // accumulated over the shortest-path tree.
    //
    // Scoring a scout target by the window it ends on is the wrong economics: a
    // scout is paid for everything it sees while travelling, so two targets
    // with identical destination windows can differ by a whole sweep's worth of
    // coverage.  Costing the trip but not crediting it is what leaves cells
    // unobserved with the fuel already spent.
    void path_gain(const vector<int> &d, const vector<int> &p, int r, bool cross,
                   vector<int> &pg) const
    {
        pg.assign(n * n, 0);
        vector<pair<int, int>> ord; // (dist, cell), so parents come first
        ord.reserve(n * n);
        for (int i = 0; i < n * n; ++i)
            if (d[i] < PLAN_INF)
                ord.push_back(make_pair(d[i], i));
        sort(ord.begin(), ord.end());
        for (size_t i = 0; i < ord.size(); ++i)
        {
            int v = ord[i].second, u = p[v];
            if (u < 0)
                continue; // the source itself: nothing collected yet
            pg[v] = pg[u] + step_gain(u / n, u % n, v / n, v % n, r, cross);
        }
    }

    // ---- free path steering -----------------------------------------------
    // Expected number of *undiscovered* tasks on each cell: the mass term of
    // the observation value without the serve weighting.  It is separated out
    // because it can be built before the per-robot dijkstras, whereas the full
    // value cannot (it needs serve_dist, which needs those dijkstras) -- and a
    // path can only be steered by something that exists when it is planned.
    vector<int> obs_mass;

    void build_obs_mass()
    {
        obs_mass.assign(n * n, 0);
        int ft = spawned_frac(now);
        for (int x = 0; x < n; ++x)
            for (int y = 0; y < n; ++y)
            {
                if ((*obj_map)[x][y] == OBJECT::WALL)
                    continue;
                int ls = last_seen[x][y];
                int m = (ls < 0) ? (1000 + ft) : (ft - spawned_frac(ls));
                if (m > 0)
                    obs_mass[idx(x, y)] = m;
            }
    }

    // edge_val[type][k*n*n + v] = mass a robot of that type brings into view by
    // stepping into cell v heading in direction k.  Row/column prefix sums make
    // each entry O(1), so the whole table costs one pass over the grid.
    vector<int> edge_val[3];

    void build_edge_val()
    {
        vector<int> pc((n + 1) * n, 0), pr((n + 1) * n, 0);
        for (int x = 0; x < n; ++x)
            for (int y = 0; y < n; ++y)
                pc[x * (n + 1) + y + 1] = pc[x * (n + 1) + y] + obs_mass[idx(x, y)];
        for (int y = 0; y < n; ++y)
            for (int x = 0; x < n; ++x)
                pr[y * (n + 1) + x + 1] = pr[y * (n + 1) + x] + obs_mass[idx(x, y)];
        for (int ty = 0; ty < 3; ++ty)
        {
            int r = ROBOT::view_range_list[ty];
            bool cross = (ROBOT::view_type_list[ty] == ROBOT::VIEWTYPE::CROSS);
            edge_val[ty].assign(4 * n * n, 0);
            for (int k = 0; k < 4; ++k)
                for (int x = 0; x < n; ++x)
                    for (int y = 0; y < n; ++y)
                    {
                        int dx = DXS[k], dy = DYS[k], g;
                        if (cross)
                        {
                            // the tip that appears ahead, plus the arm laid
                            // across the direction of travel; the cell itself
                            // was already in view before the step
                            int px = x + dx * r, py = y + dy * r;
                            int tip = in_map(px, py) ? obs_mass[idx(px, py)] : 0;
                            int arm = dx != 0 ? colsum(pc, x, y - r, y + r)
                                              : rowsum(pr, y, x - r, x + r);
                            g = tip + arm - obs_mass[idx(x, y)];
                        }
                        else
                        {
                            g = dx != 0 ? colsum(pc, x + dx * r, y - r, y + r)
                                        : rowsum(pr, y + dy * r, x - r, x + r);
                        }
                        edge_val[ty][k * n * n + idx(x, y)] = g < 0 ? 0 : g;
                    }
        }
    }

    int colsum(const vector<int> &pc, int x, int a, int b) const
    {
        if (x < 0 || x >= n)
            return 0;
        if (a < 0)
            a = 0;
        if (b > n - 1)
            b = n - 1;
        if (a > b)
            return 0;
        return pc[x * (n + 1) + b + 1] - pc[x * (n + 1) + a];
    }
    int rowsum(const vector<int> &pr, int y, int a, int b) const
    {
        if (y < 0 || y >= n)
            return 0;
        if (a < 0)
            a = 0;
        if (b > n - 1)
            b = n - 1;
        if (a > b)
            return 0;
        return pr[y * (n + 1) + b + 1] - pr[y * (n + 1) + a];
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
    // Built first so the routed paths below can be tie-broken by it.
    if (PATH_TIEBREAK)
    {
        st.build_obs_mass();
        st.build_edge_val();
    }
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
        st.dijkstra(pos, static_cast<int>(r.type), st.dist[r.id], st.par[r.id], true,
                    PATH_TIEBREAK ? &st.edge_val[static_cast<int>(r.type)] : 0);
        if (r.type != ROBOT::TYPE::DRONE)
            st.dijkstra(pos, static_cast<int>(r.type), st.dist_c[r.id], st.scratch_par, false);
        else
            st.dist_c[r.id] = st.dist[r.id];
    }

    // ---- worker/task matching --------------------------------------------
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

    // ---- fleet plan: multi-vehicle routing over the known free tasks ------
    // Measured offline (bench/plan.cpp): a planner that knows the whole map and
    // every task completes 14.5 of 16 at ~2470 energy per task, while per-tick
    // greedy matching on the same information manages 12.1 at ~3240.  The whole
    // difference is route quality, so routes are solved properly here --
    // insertion to place as many tasks as possible, then relocate / swap /
    // 2-opt to free the energy that lets one more fit -- and each worker walks
    // the first leg of its own route.  Re-solved every tick, seeded from last
    // tick's routes so that it does not churn.
    const int horizon_t = HORIZON_HARD_PER_CELL * st.n;
    const int ticks_left = horizon_t - st.now;

    struct PW
    {
        int rid, type, energy, t0;
        bool working;
    };
    vector<PW> pw;
    for (size_t i = 0; i < robots.size(); ++i)
    {
        const ROBOT &r = *robots[i];
        if (r.type == ROBOT::TYPE::DRONE || r.get_status() == ROBOT::STATUS::EXHAUSTED)
            continue;
        if (r.get_energy() < WORKER_MIN_ENERGY || st.dist_c[r.id].empty())
            continue;
        PW p;
        p.rid = r.id;
        p.type = static_cast<int>(r.type);
        p.working = (r.get_status() == ROBOT::STATUS::WORKING);
        p.energy = r.get_energy();
        p.t0 = 0;
        if (p.working)
        {
            // it still owes the REST of its current job -- charging the whole of
            // it (as this used to) understates the worker by up to a full task's
            // work every time it is mid-job, which is most of the time
            if (st.work_since.find(r.id) == st.work_since.end())
                st.work_since[r.id] = st.now;
            for (size_t q = 0; q < active_tasks.size(); ++q)
            {
                const TASK *t = active_tasks[q].get();
                if (t && !t->is_done() && t->get_assigned_robot_id() == r.id)
                {
                    int we = work_energy(*t, r.type);
                    if (we < PLAN_INF)
                    {
                        int spent = (st.now - st.work_since[r.id]) * 10;
                        int left = we - spent;
                        if (left < 10)
                            left = 10;
                        p.energy -= left;
                        p.t0 = left / 10;
                    }
                    break;
                }
            }
            if (p.energy < 0)
                p.energy = 0;
        }
        else
            st.work_since.erase(r.id);
        pw.push_back(p);
    }

    vector<vector<int>> route(pw.size()); // indices into `tasks`, in visit order

    // travel energy of one leg: from a worker's own position (from < 0) or from
    // another task's cell.  Always the CLEAN maps -- the magnet-routed ones are
    // for walking, never for costing.
    struct Leg
    {
        Scheduler::State *st;
        const vector<const TASK *> *tasks;
        const vector<PW> *pw;
        int operator()(size_t k, int from, int to) const
        {
            const TASK *t = (*tasks)[to];
            if (from < 0)
                return (*st).dist_c[(*pw)[k].rid][(*st).idx(t->coord)];
            const TASK *f = (*tasks)[from];
            return (*st).dist_from_task(f->id, (*pw)[k].type, f->coord)[(*st).idx(t->coord)];
        }
    };
    Leg leg;
    leg.st = &st;
    leg.tasks = &tasks;
    leg.pw = &pw;

    // A route is feasible when every stop is paid for in energy and finished
    // before the run ends; e_out receives its total energy.
    struct Eval
    {
        const Leg *leg;
        const vector<const TASK *> *tasks;
        const vector<PW> *pw;
        int now, horizon;
        bool operator()(size_t k, const vector<int> &seq, int *e_out) const
        {
            const PW &p = (*pw)[k];
            int e = 0, t = now + p.t0, prev = -1;
            int budget = p.energy;
            for (size_t i = 0; i < seq.size(); ++i)
            {
                int j = seq[i];
                int trav = (*leg)(k, prev, j);
                if (trav >= PLAN_INF)
                    return false;
                int we = work_energy(*(*tasks)[j], static_cast<ROBOT::TYPE>(p.type));
                if (we >= PLAN_INF || e + trav + we > budget)
                    return false;
                t += (trav + we) / 10;
                if (t > horizon - PLAN_SLACK)
                    return false;
                e += trav + we;
                prev = j;
            }
            if (e_out)
                *e_out = e;
            return true;
        }
    };
    Eval ok;
    ok.leg = &leg;
    ok.tasks = &tasks;
    ok.pw = &pw;
    ok.now = st.now;
    ok.horizon = horizon_t;

    // ---- exact plan, when the known task set is small enough ---------------
    // The local search below is insertion + relocate/swap/2-opt, and measured
    // offline against a subset-DP solver it lands ~1.1 tasks short of the true
    // optimum.  With few enough free tasks the optimum is directly computable:
    // for each worker, a DP over (subset, last visited) carrying the Pareto
    // frontier of (energy, completion time) -- the two are not interchangeable,
    // because waiting for a task to be reachable costs time but no energy --
    // then a subset partition across workers maximising the count and breaking
    // ties on total energy.
    bool exact_done = false;
    if (EXACT_MAX > 0 && !tasks.empty() && static_cast<int>(tasks.size()) <= EXACT_MAX &&
        pw.size() <= 6)
    {
        const int nfree = static_cast<int>(tasks.size());
        const int FULL = 1 << nfree;
        // per worker: best[S] = minimal energy to serve exactly S (PLAN_INF = can't)
        vector<vector<int>> bestE(pw.size(), vector<int>(FULL, PLAN_INF));
        // pareto[S * nfree + last] = frontier of (energy, finish tick)
        vector<vector<pair<int, int> > > par;
        for (size_t k = 0; k < pw.size(); ++k)
        {
            par.assign(static_cast<size_t>(FULL) * nfree, vector<pair<int, int> >());
            for (int j = 0; j < nfree; ++j)
            {
                vector<int> one(1, j);
                int e = 0;
                if (!ok(k, one, &e))
                    continue;
                par[static_cast<size_t>(1 << j) * nfree + j].push_back(
                    make_pair(e, st.now + pw[k].t0 + e / 10));
                if (e < bestE[k][1 << j])
                    bestE[k][1 << j] = e;
            }
            for (int S = 1; S < FULL; ++S)
                for (int last = 0; last < nfree; ++last)
                {
                    if (!((S >> last) & 1))
                        continue;
                    const vector<pair<int, int> > cur = par[static_cast<size_t>(S) * nfree + last];
                    if (cur.empty())
                        continue;
                    for (int j = 0; j < nfree; ++j)
                    {
                        if ((S >> j) & 1)
                            continue;
                        int legc = leg(k, last, j);
                        int we = work_energy(*tasks[j], static_cast<ROBOT::TYPE>(pw[k].type));
                        if (legc >= PLAN_INF || we >= PLAN_INF)
                            continue;
                        int S2 = S | (1 << j);
                        for (size_t c = 0; c < cur.size(); ++c)
                        {
                            int e = cur[c].first + legc + we;
                            if (e > pw[k].energy)
                                continue;
                            int t = cur[c].second + (legc + we) / 10;
                            if (t > horizon_t - PLAN_SLACK)
                                continue;
                            // dominance prune
                            vector<pair<int, int> > &v = par[static_cast<size_t>(S2) * nfree + j];
                            bool dom = false;
                            for (size_t q = 0; q < v.size() && !dom; ++q)
                                if (v[q].first <= e && v[q].second <= t)
                                    dom = true;
                            if (dom)
                                continue;
                            for (size_t q = 0; q < v.size();)
                            {
                                if (v[q].first >= e && v[q].second >= t)
                                    v.erase(v.begin() + q);
                                else
                                    ++q;
                            }
                            v.push_back(make_pair(e, t));
                            if (e < bestE[k][S2])
                                bestE[k][S2] = e;
                        }
                    }
                }
        }
        // partition: maximise served count, tie-break on total energy
        const int NEG = -1;
        vector<int> cnt(FULL, NEG), eng(FULL, PLAN_INF);
        vector<int> pick(static_cast<size_t>(FULL) * pw.size(), 0);
        cnt[0] = 0;
        eng[0] = 0;
        for (size_t k = 0; k < pw.size(); ++k)
        {
            vector<int> ncnt(FULL, NEG), neng(FULL, PLAN_INF);
            vector<int> npick(pick.size(), 0);
            for (int S = 0; S < FULL; ++S)
            {
                if (cnt[S] < 0)
                    continue;
                {   // worker k serves nothing and stays where it is
                    int e2 = eng[S];
                    if (cnt[S] > ncnt[S] || (cnt[S] == ncnt[S] && e2 < neng[S]))
                    {
                        ncnt[S] = cnt[S];
                        neng[S] = e2;
                        for (size_t q = 0; q < pw.size(); ++q)
                            npick[static_cast<size_t>(S) * pw.size() + q] =
                                pick[static_cast<size_t>(S) * pw.size() + q];
                        npick[static_cast<size_t>(S) * pw.size() + k] = 0;
                    }
                }
                int rest = (FULL - 1) & ~S;
                for (int T = rest; T; T = (T - 1) & rest)
                {
                    if (bestE[k][T] >= PLAN_INF)
                        continue;
                    int c2 = cnt[S] + __builtin_popcount(T);
                    int e2 = eng[S] + bestE[k][T];
                    int U = S | T;
                    if (c2 > ncnt[U] || (c2 == ncnt[U] && e2 < neng[U]))
                    {
                        ncnt[U] = c2;
                        neng[U] = e2;
                        for (size_t q = 0; q < pw.size(); ++q)
                            npick[static_cast<size_t>(U) * pw.size() + q] =
                                pick[static_cast<size_t>(S) * pw.size() + q];
                        npick[static_cast<size_t>(U) * pw.size() + k] = T;
                    }
                }
            }
            cnt.swap(ncnt);
            eng.swap(neng);
            pick.swap(npick);
        }
        int bestS = 0;
        for (int S = 0; S < FULL; ++S)
            if (cnt[S] > cnt[bestS] || (cnt[S] == cnt[bestS] && eng[S] < eng[bestS]))
                bestS = S;
        // reconstruct each worker's order over its assigned subset, greedily by
        // cheapest feasible extension (subsets are small; verified by ok())
        for (size_t k = 0; k < pw.size(); ++k)
        {
            int T = pick[static_cast<size_t>(bestS) * pw.size() + k];
            route[k].clear();
            while (T)
            {
                int bj = -1, be = PLAN_INF;
                for (int j = 0; j < nfree; ++j)
                {
                    if (!((T >> j) & 1))
                        continue;
                    vector<int> trial = route[k];
                    trial.push_back(j);
                    int rest = T & ~(1 << j);
                    // must stay completable: append the rest in any feasible order
                    int e = 0;
                    if (!ok(k, trial, &e))
                        continue;
                    if (rest && bestE[k][T] >= PLAN_INF)
                        continue;
                    if (e < be)
                    {
                        be = e;
                        bj = j;
                    }
                }
                if (bj < 0)
                    break;
                route[k].push_back(bj);
                T &= ~(1 << bj);
            }
        }
        exact_done = true;
    }

    vector<char> placed(tasks.size(), 0);
    if (!exact_done)
    {
        // seed from last tick's routes: same plan unless something really
        // changed, which is what keeps workers from re-targeting every tick
        map<int, int> id_to_idx;
        for (size_t j = 0; j < tasks.size(); ++j)
            id_to_idx[tasks[j]->id] = static_cast<int>(j);
        for (size_t k = 0; k < pw.size(); ++k)
        {
            map<int, vector<int>>::iterator it = st.prev_route.find(pw[k].rid);
            if (it == st.prev_route.end())
                continue;
            for (size_t q = 0; q < it->second.size(); ++q)
            {
                map<int, int>::iterator f = id_to_idx.find(it->second[q]);
                if (f == id_to_idx.end() || placed[f->second])
                    continue;
                vector<int> trial = route[k];
                trial.push_back(f->second);
                if (!ok(k, trial, 0))
                    continue;
                route[k].swap(trial);
                placed[f->second] = 1;
            }
        }
    }

    for (size_t k = 0; k < route.size() && exact_done; ++k)
        for (size_t q = 0; q < route[k].size(); ++q)
            placed[route[k][q]] = 1;

    // insertion: repeatedly take the globally cheapest feasible placement
    for (; !exact_done;)
    {
        int best_inc = PLAN_INF, bj = -1, bp = -1;
        size_t bk = 0;
        for (size_t k = 0; k < pw.size(); ++k)
        {
            int e0 = 0;
            if (!ok(k, route[k], &e0))
                continue;
            for (size_t j = 0; j < tasks.size(); ++j)
            {
                if (placed[j])
                    continue;
                for (size_t p = 0; p <= route[k].size(); ++p)
                {
                    vector<int> trial = route[k];
                    trial.insert(trial.begin() + p, static_cast<int>(j));
                    int e1 = 0;
                    if (!ok(k, trial, &e1))
                        continue;
                    if (e1 - e0 < best_inc)
                    {
                        best_inc = e1 - e0;
                        bk = k;
                        bj = static_cast<int>(j);
                        bp = static_cast<int>(p);
                    }
                }
            }
        }
        if (bj < 0)
            break;
        route[bk].insert(route[bk].begin() + bp, bj);
        placed[bj] = 1;
    }

    // local search: cheaper routes free the energy that lets one more task fit
    for (int iter = 0; iter < PLAN_ITERS && !exact_done; ++iter)
    {
        bool improved = false;
        for (size_t k = 0; k < pw.size() && !improved; ++k)
            for (size_t i = 0; i < route[k].size() && !improved; ++i)
            {
                vector<int> src = route[k];
                int j = src[i];
                src.erase(src.begin() + i);
                if (!ok(k, src, 0))
                    continue;
                for (size_t k2 = 0; k2 < pw.size() && !improved; ++k2)
                {
                    const vector<int> &base = (k2 == k) ? src : route[k2];
                    for (size_t p = 0; p <= base.size(); ++p)
                    {
                        if (k2 == k && p == i)
                            continue;
                        vector<int> trial = base;
                        trial.insert(trial.begin() + p, j);
                        int before = 0, after = 0, tmp = 0;
                        if (!ok(k2, trial, &after))
                            continue;
                        ok(k, route[k], &tmp);
                        before += tmp;
                        if (k2 != k)
                        {
                            ok(k2, route[k2], &tmp);
                            before += tmp;
                            ok(k, src, &tmp);
                            after += tmp;
                        }
                        if (after < before)
                        {
                            if (k2 != k)
                                route[k] = src;
                            route[k2] = trial;
                            improved = true;
                            break;
                        }
                    }
                }
            }
        for (size_t k = 0; k < pw.size() && !improved; ++k)
            for (size_t a = 0; a + 1 < route[k].size() && !improved; ++a)
                for (size_t b = a + 1; b < route[k].size(); ++b)
                {
                    vector<int> trial = route[k];
                    reverse(trial.begin() + a, trial.begin() + b + 1);
                    int e0 = 0, e1 = 0;
                    if (!ok(k, trial, &e1) || !ok(k, route[k], &e0))
                        continue;
                    if (e1 < e0)
                    {
                        route[k].swap(trial);
                        improved = true;
                        break;
                    }
                }
        if (!improved)
            break;
        // room may have opened up for a task nobody could afford before
        for (size_t j = 0; j < tasks.size(); ++j)
        {
            if (placed[j])
                continue;
            for (size_t k = 0; k < pw.size() && !placed[j]; ++k)
                for (size_t p = 0; p <= route[k].size(); ++p)
                {
                    vector<int> trial = route[k];
                    trial.insert(trial.begin() + p, static_cast<int>(j));
                    if (!ok(k, trial, 0))
                        continue;
                    route[k].swap(trial);
                    placed[j] = 1;
                    break;
                }
        }
    }

    // ---- hand the first leg of each route to its worker --------------------
    map<int, int> new_owner;
    for (int i = 0; i <= max_id; ++i)
        st.assigned[i] = -1;
    st.prev_route.clear();
    for (size_t k = 0; k < pw.size(); ++k)
    {
        vector<int> ids;
        for (size_t q = 0; q < route[k].size(); ++q)
            ids.push_back(tasks[route[k][q]]->id);
        st.prev_route[pw[k].rid] = ids;
        if (pw[k].working || route[k].empty())
            continue;
        const ROBOT &r = *by_id[pw[k].rid];
        int j = route[k][0];
        int travel = st.dist_c[r.id][st.idx(tasks[j]->coord)];
        // Waiting clock, kept from the greedy matcher because it measured
        // positive: a far task waits a bounded time for a cheaper server to
        // free up, or for a spawn that merges into the same trip, before the
        // fleet commits energy to the march.  Removing it costs completions.
        bool waited_out = (st.now - st.first_seen[tasks[j]->id]) > STARVE_AGE;
        // When to stop hoarding and commit.  The fixed endgame window puts that
        // moment at t=800 for everyone, which is exactly where a full worker's
        // 12000 energy stops fitting in the 1200 ticks it has left.  That is the
        // real rule, and it is per worker: hold while energy is the scarce
        // resource and information is still improving; spend once the energy
        // would otherwise go unused.  A worker that keeps spending optimally
        // sits on the crossover, so this reproduces t=800 for an untouched
        // worker and adapts for one that got drawn in early or delayed.
        bool go = (static_cast<long long>(r.get_energy()) >=
                   static_cast<long long>(10) * ticks_left);
        if (travel > WORKER_TRAVEL_CAP && !waited_out && !go && !sole_server[j])
            continue;
        new_owner[tasks[j]->id] = r.id;
        st.assigned[r.id] = tasks[j]->id;
        Coord pos = (r.get_status() == ROBOT::STATUS::MOVING) ? r.get_target_coord() : r.get_coord();
        st.next_step[r.id] = st.first_step(st.par[r.id], pos, tasks[j]->coord);
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
        // Two-phase pacing.  The workers do not act until their energy stops
        // fitting in the time left (about t=800), so *within* that window it
        // makes no difference when a cell is first seen -- but everything seen
        // after it is a task the fleet may not reach.  So the first pass is
        // compressed into the window before the workers commit, and the rest of
        // the fuel is spread over the re-sweep that catches the late spawns.
        int floor_energy = DRONE_CAMERA_FLOOR;
        if (DRONE_PACE_T > 0 && st.now < DRONE_PACE_T)
        {
            int e0 = st.init_energy[r.id];
            double frac = static_cast<double>(st.now) / static_cast<double>(DRONE_PACE_T);
            int budget_floor = e0 - DRONE_BURST -
                               static_cast<int>((e0 - DRONE_BURST - DRONE_CAMERA_FLOOR) * frac);
            floor_energy = max(DRONE_CAMERA_FLOOR, budget_floor);
        }
        // A parked drone is a 5x5 camera, which is only worth anything while
        // there are still spawns left to catch.  Once the dispatcher is done
        // (spawned_frac == 1000) the camera can only re-see cells it is already
        // seeing, so the reserve keeping it alive is pure waste -- spend it.
        int keep = floor_energy;
        if (st.spawned_frac(st.now) >= 1000)
            keep = min(keep, DRONE_TAIL_FLOOR);
        if (r.get_energy() - DRONE_STEP_EST < keep)
            continue; // parked as camera

        // Value per unit of energy, not raw value: walking half the map to
        // refresh one window costs more than the find is worth.  The +K
        // offset keeps an adjacent cell from winning on a rounding artefact.
        int best_ratio = 0;
        Coord chosen = pos;
        vector<int> pgain;
        if (SCOUT_PATHVAL)
            st.path_gain(d, st.par[r.id], viewr, false, pgain);
        for (int x = half.first; x <= half.second; ++x)
            for (int y = 0; y < st.n; ++y)
            {
                int dd = d[st.idx(x, y)];
                if (dd >= PLAN_INF || dd > r.get_energy() - DRONE_CAMERA_FLOOR)
                    continue;
                if (Coord(x, y) == pos)
                    continue;
                int g = SCOUT_PATHVAL ? pgain[st.idx(x, y)]
                                      : st.window_gain(x, y, viewr, false);
                if (g <= 0)
                    continue;
                int ratio = static_cast<int>(static_cast<long long>(g) * 1000 / (dd + SCOUT_K));
                if (ratio > best_ratio)
                {
                    best_ratio = ratio;
                    chosen = Coord(x, y);
                }
            }

        map<int, Coord>::iterator gi = st.drone_goal.find(r.id);
        bool need_new = true;
        if (gi != st.drone_goal.end())
        {
            Coord g = gi->second;
            if (!(g == pos) && d[st.idx(g)] < PLAN_INF)
            {
                need_new = false;
                // A goal is normally held until reached -- re-picking every tick
                // oscillates, because approaching a target observes away its own
                // value.  But a commitment can also go stale for a reason that
                // has nothing to do with this drone: another robot sweeps the
                // target's window first.  Abandon it only when something else is
                // now worth clearly more, which is the case the oscillation
                // argument does not cover.
                if (SCOUT_RECOMMIT > 0)
                {
                    int gg = SCOUT_PATHVAL ? pgain[st.idx(g)]
                                           : st.window_gain(g.x, g.y, viewr, false);
                    long long gr = static_cast<long long>(gg) * 1000 / (d[st.idx(g)] + SCOUT_K);
                    if (static_cast<long long>(best_ratio) * 100 > gr * SCOUT_RECOMMIT)
                        need_new = true;
                }
            }
        }
        if (need_new)
        {
            if (best_ratio < SCOUT_MIN_RATIO || chosen == pos)
            {
                st.drone_goal.erase(r.id);
                continue;
            }
            st.drone_goal[r.id] = chosen;
        }
        st.next_step[r.id] = st.first_step(st.par[r.id], pos, st.drone_goal[r.id]);
    }



    if (true)
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
            // Same crossover as the service gate: energy is only free to spend
            // on looking once it could not all be spent on serving anyway.
            if (static_cast<long long>(r.get_energy()) <
                static_cast<long long>(10) * (horizon_t - st.now))
                continue;
            int floor_e = (st.now >= PATROL_LATE_T) ? PATROL_LATE_ENERGY : PATROL_MIN_ENERGY;
            if (st.assigned[r.id] != -1 || r.get_energy() < floor_e || st.dist_c[r.id].empty())
                continue;
            Coord pos = (r.get_status() == ROBOT::STATUS::MOVING) ? r.get_target_coord() : r.get_coord();
            const vector<int> &d = st.dist_c[r.id];
            int viewr = ROBOT::view_range_list[static_cast<size_t>(r.type)];
            bool cross = (ROBOT::view_type_list[static_cast<size_t>(r.type)] == ROBOT::VIEWTYPE::CROSS);
            int range = max(0, r.get_energy() - floor_e);
            int best_ratio = 0;
            Coord best = pos;
            vector<int> pgain;
            if (SCOUT_PATHVAL)
                st.path_gain(st.dist[r.id], st.par[r.id], viewr, cross, pgain);
            for (int x = 0; x < st.n; ++x)
                for (int y = 0; y < st.n; ++y)
                {
                    int dd = d[st.idx(x, y)];
                    if (dd >= PLAN_INF || dd > range || Coord(x, y) == pos)
                        continue;
                    int g = SCOUT_PATHVAL ? pgain[st.idx(x, y)]
                                          : st.window_gain(x, y, viewr, cross);
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
