#ifndef SIMULATER_H_
#define SIMULATER_H_

// #define VERBOSE

#include <iostream>
#include <vector>
#include <set>
#include <array>
#include <iomanip>
#include <functional>
#include <cstdio>
#include <chrono>
#include <conio.h>
#include <sstream>
#include <fstream>

// #define VERBOSE

using namespace std;

constexpr int INFINITE = std::numeric_limits<int>::max();

// OBJECT type
enum class OBJECT : int
{
    EMPTY = 0b0000,
    ROBOT = 0b0001,
    TASK = 0b0010,
    ROBOT_AND_TASK = 0b0011,
    WALL = 0b0100,
    UNKNOWN = 0b1000
};

string to_string(OBJECT obj);
ostream &operator<<(ostream &o, OBJECT obj);
OBJECT operator|(OBJECT lhs, OBJECT rhs);
OBJECT operator&(OBJECT lhs, OBJECT rhs);
OBJECT &operator|=(OBJECT &lhs, OBJECT rhs);
OBJECT &operator&=(OBJECT &lhs, OBJECT rhs);
OBJECT operator~(OBJECT obj);

// OBJECT type end

class MAP;
class TASK;
class Scheduler;
class TASKDISPATCHER;

class Coord
{
public:
    int x;
    int y;
    constexpr Coord() : x{-1}, y{-1} {}
    constexpr Coord(int xx, int yy) : x{xx}, y{yy} {}
    friend std::ostream &operator<<(std::ostream &o, const Coord &coord)
    {
        stringstream s;
        s << '(' << setw(2) << coord.x << setw(0) << ", " << setw(2) << coord.y << setw(0) << ')';
        o << s.str();
        return o;
    }
    Coord operator+(const Coord &rhs) const { return {(*this).x + rhs.x, (*this).y + rhs.y}; }
    Coord operator-(const Coord &rhs) const { return {(*this).x - rhs.x, (*this).y - rhs.y}; }
    bool operator==(const Coord &rhs) const { return (*this).x == rhs.x && rhs.y == (*this).y; }
    bool operator!=(const Coord &rhs) const { return !((*this) == rhs); }
    bool operator<(const Coord &rhs) const { return (this->x == rhs.x) ? this->y < rhs.y : this->x < rhs.x; }
};

class TIMER
{
public:
    chrono::nanoseconds time_elapsed = std::chrono::high_resolution_clock::duration::zero();
    void start() { start_time = now(); }
    void stop() { time_elapsed += now() - start_time; }
    chrono::high_resolution_clock::time_point now() { return chrono::high_resolution_clock::now(); }
    chrono::high_resolution_clock::time_point start_time;
};

class ROBOT : public enable_shared_from_this<ROBOT>
{
    friend class MAP;

public:
    // enum types
    enum class TYPE
    {
        DRONE,
        CATERPILLAR,
        WHEEL,
    };
    friend string to_string(TYPE type)
    {
        string str;
        switch (type)
        {
        case TYPE::DRONE:
            str = "DRONE";
            break;
        case TYPE::CATERPILLAR:
            str = "CATERPILLAR";
            break;
        case TYPE::WHEEL:
            str = "WHEEL";
            break;
        default:
            str = "Invalid";
        }
        return str;
    }
    friend ostream &operator<<(ostream &o, TYPE type)
    {
        o << to_string(type);
        return o;
    }
    enum class VIEWTYPE
    {
        CROSS,
        SQUARE
    };
    enum class STATUS
    {
        IDLE,
        WORKING,
        MOVING,
        EXHAUSTED
    };
    friend string to_string(STATUS status)
    {
        string str;
        switch (status)
        {
        case STATUS::IDLE:
            str = "IDLE";
            break;
        case STATUS::WORKING:
            str = "WORKING";
            break;
        case STATUS::MOVING:
            str = "MOVING";
            break;
        case STATUS::EXHAUSTED:
            str = "EXHAUSTED";
            break;
        default:
            str = "Invalid";
        }
        return str;
    }
    friend ostream &operator<<(ostream &o, STATUS status)
    {
        o << to_string(status);
        return o;
    }
    enum class ACTION
    {
        UP,
        DOWN,
        LEFT,
        RIGHT,
        HOLD
    };
    friend string to_string(ACTION action)
    {
        string str;
        switch (action)
        {
        case ACTION::UP:
            str = "UP";
            break;
        case ACTION::DOWN:
            str = "DOWN";
            break;
        case ACTION::LEFT:
            str = "LEFT";
            break;
        case ACTION::RIGHT:
            str = "RIGHT";
            break;
        case ACTION::HOLD:
            str = "HOLD";
            break;
        default:
            str = "Invalid";
        }
        return str;
    }
    friend ostream &operator<<(ostream &o, const ACTION &action)
    {
        o << to_string(action);
        return o;
    }

    // Static constants

    static constexpr int NUM_ROBOT_TYPE = 3;
    static constexpr int ROBOT_ENERGY_PER_TICK = 10;
    static constexpr int TASK_PROGRESS_PER_TICK = ROBOT_ENERGY_PER_TICK;
    static constexpr int view_range_list[] = {2, 1, 1};
    static constexpr VIEWTYPE view_type_list[] = {VIEWTYPE::SQUARE, VIEWTYPE::SQUARE, VIEWTYPE::CROSS}; // 1: cross range, 2: square range
    static constexpr int energy_per_tick_list[] = {ROBOT_ENERGY_PER_TICK, ROBOT_ENERGY_PER_TICK, ROBOT_ENERGY_PER_TICK};

    // Constant variables

    const int id;
    const TYPE type;

    // Get methods

    const Coord &get_coord() const noexcept { return this->coord; }
    const STATUS &get_status() const noexcept { return this->status; }
    const Coord &get_target_coord() const noexcept { return target_coord; }
    int get_energy() const noexcept { return energy; }

    // Public methods

    bool start_moving(ACTION action);
    bool start_working(weak_ptr<TASK> task);
    int move();
    int work();

    // Constructor
    ROBOT(const Coord &coord, const TYPE type, int id, MAP &map, int energy) : coord(coord), type(type), id(id), map(map), status(STATUS::IDLE), energy(energy) {}

private:
    // Private variables
    MAP &map;
    Coord coord;
    STATUS status;
    int energy;
    weak_ptr<TASK> assigned_task;
    Coord target_coord = {-1, -1};
    int remain_progress = 0;

    // Private methods
    int consume_energy();
    int get_remain_progress() const noexcept { return remain_progress; }
};

class TASK
{
    friend class MAP;
    friend std::ostream &operator<<(std::ostream &o, const TASK &task);
    friend bool ROBOT::start_working(weak_ptr<TASK> task);
    friend int ROBOT::work();

public:
    // Constant variables

    const Coord coord;
    const array<int, ROBOT::NUM_ROBOT_TYPE> task_cost;
    const int id;

    // Public Constructor

    TASK(Coord coord, int id, MAP &map) : TASK(coord, id, {DRONE_DEFAULT_COST, rand() % CATERPILLAR_COST_EXCLUSIVE_UPPER_BOUND + CATERPILLAR_COST_MIN, rand() % WHEEL_COST_EXCLUSIVE_UPPER_BOUND + WHEEL_COST_MIN}, map) {}

    // Public methods

    bool is_done() const { return done; }
    int get_assigned_robot_id() const { return (assigned_robot.expired()) ? -1 : assigned_robot.lock()->id; }
    /* Get task cost by type */
    int get_cost(ROBOT::TYPE type) const { return task_cost[static_cast<size_t>(type)]; }

private:
    // Private Constructor
    TASK(Coord coord, int id, array<int, ROBOT::NUM_ROBOT_TYPE> costs, MAP &map) : coord(coord), id(id), task_cost(costs), map(map) {}

    // Private variables

    bool done = false;
    weak_ptr<ROBOT> assigned_robot;
    MAP &map;

    // Private static constants
    static constexpr int DRONE_DEFAULT_COST = INFINITE;
    static constexpr int CATERPILLAR_COST_EXCLUSIVE_UPPER_BOUND = 100;
    static constexpr int CATERPILLAR_COST_MIN = 50;
    static constexpr int WHEEL_COST_EXCLUSIVE_UPPER_BOUND = 200;
    static constexpr int WHEEL_COST_MIN = 0;
};

class TASKDISPATCHER
{
public:
    TASKDISPATCHER(MAP &map, int time_max) : map(map), time_max(time_max)
    {
        next_task_arrival_time = time_max / 4;
    }

    bool try_dispatch(int current_time);

private:
    MAP &map;
    int next_task_arrival_time;
    const int time_max;
};

class MAP
{
    friend bool TASKDISPATCHER::try_dispatch(int current_time);

public:
    // Constant variables
    const int map_size;
    const int num_total_task;
    const int wall_density;
    const int time_max;

    // Constructor
    MAP(int map_size, int num_robot, int num_initial_task, int num_total_task, int wall_density, int robot_energy)
        : map_size(map_size), time_max(map_size * 100), wall_density(wall_density), num_total_task(num_total_task)
    {
        generate_map(num_robot, num_initial_task, robot_energy);
    }

    // Get methods
    Coord get_random_empty_coord() const
    {
        Coord coord = {rand() % map_size, rand() % map_size};
        while (object_at(coord) != OBJECT::EMPTY)
            coord = {rand() % map_size, rand() % map_size};
        return coord;
    };
    vector<shared_ptr<ROBOT>> &get_robots() { return robots; }
    vector<shared_ptr<TASK>> &get_tasks() { return tasks; }
    vector<vector<vector<int>>> &get_known_cost_map() { return known_cost_map; }
    vector<vector<OBJECT>> &get_known_object_map() { return known_object_map; }
    vector<shared_ptr<TASK>> &get_active_tasks() { return active_tasks; }
    int get_exhausted_robot_num() { return exhausted_robot_num; }
    int get_completed_task_num() { return completed_task_num; }
    int get_cost(const Coord &coord, ROBOT::TYPE type) const { return known_cost_at(coord, type); }
    int get_robot_num_at(int x, int y) const { return robot_num_map[x][y]; }
    int get_robot_num_at(const Coord &coord) const { return robot_num_map[coord.x][coord.y]; }

    // Public method for Robot
    weak_ptr<ROBOT> create_robot(ROBOT::TYPE type, int robot_energy)
    {
        robots.emplace_back(make_shared<ROBOT>(get_random_empty_coord(), type, static_cast<int>(robots.size()), *this, robot_energy));
        auto robot = robots.back();

        object_at(robot->coord) |= OBJECT::ROBOT;
        robot_num_at(robot->coord) += 1;
        return robot;
    }
    bool is_in(const Coord &coord) const { return coord.x >= 0 && coord.y >= 0 && coord.x < map_size && coord.y < map_size; }
    bool start_robot_moving(ROBOT &robot, ROBOT::ACTION action)
    {
        if (action == ROBOT::ACTION::HOLD)
        {
#ifdef VERBOSE
            cout << "Robot " << robot.id << " hold at " << robot.coord << endl;
#endif
            return true;
        }
        static const Coord direction[] = {{0, 1}, {0, -1}, {-1, 0}, {1, 0}};
        Coord target_coord = robot.coord + direction[static_cast<size_t>(action)];
        if (!is_in(target_coord))
        {
            cout << "Robot " << robot.id << " try to leave the map. ( From " << robot.coord << " to " << target_coord << " )" << endl;
            return false;
        }

        if (object_at(target_coord) == OBJECT::WALL)
        {
            cout << "Robot " << robot.id << " try to move to Wall. ( From " << robot.coord << " to " << target_coord << " )" << endl;
            return false;
        }
#ifdef VERBOSE
        cout << "Robot " << robot.id << " start moving " << action << " from " << robot.coord << " to " << robot.target_coord << endl;
#endif

        robot.target_coord = target_coord;
        robot.remain_progress = cost_at(robot.coord, robot.type) / 2;
        robot.status = ROBOT::STATUS::MOVING;
        return true;
    }
    void move_robot(ROBOT &robot)
    {
        if (robot.status == ROBOT::STATUS::MOVING && robot.energy > 0 && robot.remain_progress <= 0 && robot.coord != robot.target_coord)
        {
            if ((robot_num_at(robot.coord) -= 1) == 0)
                object_at(robot.coord) &= ~OBJECT::ROBOT;
            if ((robot_num_at(robot.target_coord) += 1) == 1)
                object_at(robot.target_coord) |= OBJECT::ROBOT;
            robot.remain_progress += cost_at(robot.target_coord, robot.type);
            robot.coord = robot.target_coord;
        }
    }
    friend int ROBOT::consume_energy();

    // Public method for Task
    weak_ptr<TASK> create_task()
    {
        Coord coord = get_random_empty_coord();
        tasks.emplace_back(make_shared<TASK>(coord, static_cast<int>(tasks.size()), *this));
        auto task = tasks.back();
        object_at(coord) |= OBJECT::TASK;
        return task;
    }
    weak_ptr<TASK> task_at(const Coord &coord)
    {
        auto it = tasks.begin();
        for (; it != tasks.end(); ++it)
        {
            if ((*it)->coord == coord && !(*it)->is_done())
                break;
        }
        if (it == tasks.end())
            return weak_ptr<TASK>();
        else
            return (*it);
    }
    set<Coord> observed_coord_by_robot()
    {
        set<Coord> observed_coord_set;
        int viewrange;
        ROBOT::VIEWTYPE viewtype;
        int x;
        int y;
        for (const auto robot : robots)
        {
            if (robot->status == ROBOT::STATUS::EXHAUSTED)
                continue;

            viewrange = ROBOT::view_range_list[static_cast<size_t>(robot->type)];
            viewtype = ROBOT::view_type_list[static_cast<size_t>(robot->type)];
            x = robot->coord.x;
            y = robot->coord.y;

            if (viewtype == ROBOT::VIEWTYPE::CROSS)
            {
                for (int xx = max(x - viewrange, 0); xx < min(x + viewrange + 1, map_size); ++xx)
                {
                    observed_coord_set.emplace(xx, y);
                }
                for (int yy = max(y - viewrange, 0); yy < min(y + viewrange + 1, map_size); ++yy)
                {
                    observed_coord_set.emplace(x, yy);
                }
            }
            else if (viewtype == ROBOT::VIEWTYPE::SQUARE)
            {
                for (int xx = max(x - viewrange, 0); xx < min(x + viewrange + 1, map_size); ++xx)
                {
                    for (int yy = max(y - viewrange, 0); yy < min(y + viewrange + 1, map_size); ++yy)
                    {
                        observed_coord_set.emplace(xx, yy);
                    }
                }
            }
        }
        return observed_coord_set;
    }
    set<Coord> update_coords(const set<Coord> &observed_coord_set)
    {
        set<Coord> updated_coord_set;
        updated_coord_set = previous_update;
        for (auto &coord : observed_coord_set)
        {
            auto &known_object = known_object_at(coord);
            auto &object = object_at(coord);
            if (known_object != object)
            {
                if (bool(~known_object & object & OBJECT::TASK))
                {
                    active_tasks.emplace_back(task_at(coord));
#ifdef VERBOSE
                    cout << "Task " << active_tasks.back()->id << " is found at " << coord << endl;
#endif
                }
                if (known_object == OBJECT::UNKNOWN)
                {
                    known_cost_at(coord) = cost_at(coord);
                }
                known_object = object;
                updated_coord_set.insert(coord);
            }
        }
        previous_update.clear();
        return updated_coord_set;
    }
    bool complete_task(weak_ptr<TASK> weak_ptr_task)
    {
        auto task = weak_ptr_task.lock();
        if (task->assigned_robot.lock()->remain_progress > 0)
        {
            cout << "Task " << task->id << task->coord << "is not complete" << endl;
            return false;
        }
#ifdef VERBOSE
        cout << "Task " << task->id << " at " << task->coord << " is completed by Robot " << task->assigned_robot.lock()->id << endl;
#endif // VERBOSE
        task->done = true;
        for (auto it = active_tasks.begin(); it != active_tasks.end(); ++it)
        {
            if ((*it) == task)
            {
                active_tasks.erase(it);
                break;
            }
        }
        known_object_at(task->coord) = object_at(task->coord) &= ~OBJECT::TASK;
        previous_update.insert(task->coord);
        ++completed_task_num;

        return true;
    }

    // Print methods

    void print_base(function<void(int, int)> f) const;
    void print_cost_map(ROBOT::TYPE type) const;
    void print_object_map() const;
    void print_known_object_map() const;
    void print_robot_summary() const;
    void print_task_summary() const;

private:
    // Private variables
    int current_time = 0;
    int exhausted_robot_num = 0;
    int completed_task_num = 0;
    vector<shared_ptr<ROBOT>> robots;
    vector<shared_ptr<TASK>> tasks;
    vector<vector<vector<int>>> cost_map;
    vector<vector<OBJECT>> object_map;
    vector<vector<int>> robot_num_map;
    vector<vector<vector<int>>> known_cost_map;
    vector<vector<OBJECT>> known_object_map;
    vector<shared_ptr<TASK>> active_tasks;
    set<Coord> previous_update;

    // At methods
    OBJECT &object_at(int x, int y) { return object_map[x][y]; }
    const OBJECT object_at(int x, int y) const { return object_map[x][y]; }
    OBJECT &object_at(Coord coord) { return object_map[coord.x][coord.y]; }
    const OBJECT object_at(Coord coord) const { return object_map[coord.x][coord.y]; }
    OBJECT &known_object_at(Coord coord) { return known_object_map[coord.x][coord.y]; }
    const OBJECT known_object_at(Coord coord) const { return known_object_map[coord.x][coord.y]; }
    OBJECT &known_object_at(int x, int y) { return known_object_map[x][y]; }
    const OBJECT known_object_at(int x, int y) const { return known_object_map[x][y]; }
    vector<int> &cost_at(Coord coord) { return cost_map[coord.x][coord.y]; }
    const vector<int> &cost_at(Coord coord) const { return cost_map[coord.x][coord.y]; }
    int &cost_at(Coord coord, ROBOT::TYPE type) { return cost_at(coord)[static_cast<size_t>(type)]; }
    int &cost_at(const ROBOT &robot) { return cost_at(robot.coord, robot.type); }
    int cost_at(Coord coord, ROBOT::TYPE type) const { return cost_at(coord)[static_cast<size_t>(type)]; }
    int cost_at(const ROBOT &robot) const { return cost_at(robot.coord, robot.type); }
    vector<int> &known_cost_at(const Coord &coord) { return known_cost_map[coord.x][coord.y]; }
    const vector<int> &known_cost_at(const Coord &coord) const { return known_cost_map[coord.x][coord.y]; }
    int &known_cost_at(const Coord &coord, ROBOT::TYPE type) { return known_cost_map[coord.x][coord.y][static_cast<size_t>(type)]; }
    int known_cost_at(const Coord &coord, ROBOT::TYPE type) const { return known_cost_map[coord.x][coord.y][static_cast<size_t>(type)]; }
    int &robot_num_at(const Coord &coord) { return robot_num_map[coord.x][coord.y]; }

    // Map generate
    void generate_map(int num_robot, int num_initial_task, int robot_energy)
    {
        // resize map;
        cost_map = vector<vector<vector<int>>>(map_size, vector<vector<int>>(map_size, vector<int>(ROBOT::NUM_ROBOT_TYPE)));
        object_map = vector<vector<OBJECT>>(map_size, vector<OBJECT>(map_size, OBJECT::EMPTY));
        robot_num_map = vector<vector<int>>(map_size, vector<int>(map_size, 0));
        known_cost_map = vector<vector<vector<int>>>(map_size, vector<vector<int>>(map_size, vector<int>(ROBOT::NUM_ROBOT_TYPE, -1)));
        known_object_map = vector<vector<OBJECT>>(map_size, vector<OBJECT>(map_size, OBJECT::UNKNOWN));

        // generate terrein
        int droneCost = (rand() % 40 + 60) * 2;
        int tempCost;
        for (int xx = 0; xx < map_size; ++xx)
        {
            for (int yy = 0; yy < map_size; ++yy)
            {
                cost_map[xx][yy][0] = droneCost;
                tempCost = (rand() % 200);
                cost_map[xx][yy][1] = tempCost * 2 + 100;
                cost_map[xx][yy][2] = tempCost * 4 + 50;
            }
        }

        // generate walls
        int temp = 0;
        for (int i = 0; i < map_size * map_size * wall_density / 100; ++i)
        {
            auto coord = get_random_empty_coord();
            int x = coord.x;
            int y = coord.y;
            object_map[x][y] = OBJECT::WALL;
            for (int t = 0; t < ROBOT::NUM_ROBOT_TYPE; ++t)
            {
                cost_map[x][y][t] = INFINITE;
                object_map[x][y] = OBJECT::WALL;
            }
        }

        // generate tasks
        for (int i = 0; i < num_initial_task; ++i)
        {
            create_task();
        }

        // generate robots
        for (int i = 0; i < num_robot; ++i)
        {
            ROBOT::TYPE type = ROBOT::TYPE(i % ROBOT::NUM_ROBOT_TYPE);
            create_robot(type, robot_energy);
        }

        // update known map
        update_coords(observed_coord_by_robot());
    }
};

#endif SIMULATER_H_