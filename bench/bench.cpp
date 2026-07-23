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

    constexpr int MAP_SIZE = 20;
    constexpr int NUM_ROBOT = 6;
    const int NUM_INITIAL_TASKS = NUM_MAX_TASKS / 2;
    constexpr int WALL_DENSITY = 20;
    constexpr int TIME_MAX = MAP_SIZE * 100;
    constexpr int ROBOT_ENERGY = TIME_MAX * 6;
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

    // instrumentation: per-task spawn/discovery times + affordability windows
    vector<int> spawn_time, disc_time, first_affordable, last_affordable, affordable_ticks;
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
    cout << seed << "," << NUM_MAX_TASKS << "," << created << "," << discovered
         << "," << completed << "," << map.get_exhausted_robot_num() << "," << time
         << "," << worker_energy << "," << drone_energy << endl;

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
        cout << "unknown cells at end: " << unknown_cells << "/" << MAP_SIZE * MAP_SIZE << endl;
        {
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
