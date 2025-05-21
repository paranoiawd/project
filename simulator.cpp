#include "simulator.h"

string bool_str(bool b) { return (b) ? "True" : "False"; }
template <typename T>
string bool_str(T b) { return bool_str(bool(b)); }

// OBJECT type
string to_string(OBJECT obj)
{
    string str;
    switch (obj)
    {
    case OBJECT::EMPTY:
        str = "EMPTY";
        break;
    case OBJECT::ROBOT:
        str = "ROBOT";
        break;
    case OBJECT::TASK:
        str = "TASK";
        break;
    case OBJECT::ROBOT_AND_TASK:
        str = "ROBOT_AND_TASK";
        break;
    case OBJECT::WALL:
        str = "WALL";
        break;
    case OBJECT::UNKNOWN:
        str = "UNKNOWN";
        break;
    default:
        str = "Invalid";
    }
    return str;
}
ostream &operator<<(ostream &o, OBJECT obj)
{
    o << to_string(obj);
    return o;
}
OBJECT operator|(OBJECT lhs, OBJECT rhs) { return OBJECT(static_cast<int>(lhs) | static_cast<int>(rhs)); }
OBJECT operator&(OBJECT lhs, OBJECT rhs) { return OBJECT(static_cast<int>(lhs) & static_cast<int>(rhs)); }
OBJECT &operator|=(OBJECT &lhs, OBJECT rhs) { return lhs = lhs | rhs; }
OBJECT &operator&=(OBJECT &lhs, OBJECT rhs) { return lhs = lhs & rhs; }
OBJECT operator~(OBJECT obj) { return OBJECT(~static_cast<int>(obj)); }
// OBJECT type end

// ROBOT
bool ROBOT::start_moving(ACTION action) { return this->map.start_robot_moving(*this, action); };
bool ROBOT::start_working(weak_ptr<TASK> task)
{
    if (coord != task.lock()->coord)
    {
        cout << "Robot " << id << coord << " is not at task " << task.lock()->id << task.lock()->coord << endl;
        return false;
    }
    if (!task.lock()->assigned_robot.expired())
    {
        cout << "Task " << task.lock()->id << " is already assigned to Robot " << task.lock()->assigned_robot.lock()->id << endl;
    }
    assigned_task = task;
    auto r = shared_from_this();
    task.lock()->assigned_robot = r;
    remain_progress = task.lock()->task_cost[static_cast<size_t>(type)];
    status = ROBOT::STATUS::WORKING;
#ifdef VERBOSE
    cout << "Task " << task.lock()->id << " at " << task.lock()->coord << " is assigned to Robot " << id << endl;
#endif // VERBOSE

    return true;
}
int ROBOT::move()
{
    if (status != ROBOT::STATUS::MOVING)
    {
        cout << "Robot " << id << " is not in moving." << endl;
        return -1;
    }
    remain_progress -= energy_per_tick_list[static_cast<size_t>(type)];
    if (coord != target_coord && remain_progress <= 0)
    {
#ifdef VERBOSE
        cout << "Robot " << id << " is leaving from " << coord << " to " << target_coord << endl;
#endif
        map.move_robot(*this);
    }

    if (coord == target_coord && remain_progress <= 0)
    {
#ifdef VERBOSE
        cout << "Robot " << id << " arrived at " << coord << endl;
#endif
        status = STATUS::IDLE;
        remain_progress = 0;
    }
    consume_energy();
    return remain_progress;
}
int ROBOT::work()
{
    if (status != ROBOT::STATUS::WORKING)
    {
        cout << "Robot " << id << " is not in working." << endl;
        return -1;
    }
    remain_progress -= energy_per_tick_list[static_cast<size_t>(type)];
    consume_energy();
    if (remain_progress <= 0)
    {
        remain_progress = 0;
        map.complete_task(assigned_task);
        status = STATUS::IDLE;
        assigned_task.reset();
    }
    else if (status == ROBOT::STATUS::EXHAUSTED)
    {
        assigned_task.lock()->assigned_robot.reset();
    }
    return remain_progress;
}
int ROBOT::consume_energy()
{
    energy -= energy_per_tick_list[static_cast<size_t>(type)];
    if (energy <= 0)
    {
        energy = 0;
        status = STATUS::EXHAUSTED;
        map.exhausted_robot_num += 1;
    }
    return energy;
}
// ROBOT end

// TASKDISPATCHER
bool TASKDISPATCHER::try_dispatch(int current_time)
{
    if (current_time >= next_task_arrival_time && map.num_total_task > map.tasks.size())
    {
        auto task = map.create_task();
        next_task_arrival_time += time_max / map.num_total_task;
#ifdef VERBOSE
        cout << "New task " << task.lock()->id << task.lock()->coord << " has been created" << endl;
#endif
        return true;
    }
    else
        return false;
}
// TASKDISPATCHER end

// MAP

// Print methods
void MAP::print_base(function<void(int, int)> f) const
{
    cout << "  ";
    for (int i = 0; i < map_size * 4 + 1; ++i)
    {
        cout << "-";
    }
    cout << endl;
    for (int y = map_size - 1; y >= 0; --y)
    {

        cout << setw(2) << y << "|";
        for (int x = 0; x < map_size; ++x)
        {
            f(x, y);
        }
        cout << endl;
        cout << "  ";
        for (int i = 0; i < map_size * 4 + 1; ++i)
        {
            cout << "-";
        }
        cout << endl;
    }
    cout << "  ";
    for (int i = 0; i < map_size; ++i)
    {
        cout << setw(4) << i;
    }
    cout << endl
         << endl;
}
void MAP::print_cost_map(ROBOT::TYPE type) const
{
    cout << "Cost map for " << type << endl;
    auto f = [this, type](int x, int y) -> void
    {
        auto obj = this->object_at(x, y);
        if (obj == OBJECT::WALL)
            cout << "WAL|";
        else
            cout << setw(3) << this->cost_at({x, y}, type) << "|";
    };
    print_base(f);
}
void MAP::print_object_map() const
{
    cout << "Object map" << endl;
    auto f = [this](int x, int y) -> void
    {
        auto obj = this->object_at(x, y);
        if (obj == OBJECT::EMPTY)
            cout << "   |";
        else if (obj == OBJECT::ROBOT)
        {
            cout << "R";
            Coord coord(x, y);
            int robot_num = this->get_robot_num_at(coord);
            if (robot_num > 1)
            {
                cout << "S" << robot_num;
            }
            else
            {
                for (auto it = robots.begin(); it != robots.end(); ++it)
                {
                    if ((*it)->get_coord() == coord)
                    {
                        cout << to_string((*it)->type)[0] << (*it)->id;
                        break;
                    }
                }
            }
            cout << "|";
        }
        else if (obj == OBJECT::TASK)
        {
            for (auto it = tasks.begin(); it != tasks.end(); ++it)
            {
                if ((*it)->coord == Coord(x, y) && !(*it)->is_done())
                {
                    cout << "T" << setfill('0') << setw(2) << (*it)->id << setw(0) << setfill(' ') << '|';
                    break;
                }
            }
        }
        else if (obj == OBJECT::ROBOT_AND_TASK)
        {
            vector<shared_ptr<ROBOT>> result;
            Coord coord(x, y);
            cout << "T";
            int robot_num = this->get_robot_num_at(coord);
            if (robot_num > 1)
            {
                cout << "S" << robot_num;
            }
            else
            {
                for (auto it = robots.begin(); it != robots.end(); ++it)
                {
                    if ((*it)->get_coord() == coord)
                    {
                        cout << to_string((*it)->type)[0] << (*it)->id;
                        break;
                    }
                }
            }
            cout << "|";
        }
        else
            cout << to_string(obj).substr(0, 3) << "|";
    };
    print_base(f);
}
void MAP::print_known_object_map() const
{
    cout << "Known object map" << endl;
    auto f = [this](int x, int y) -> void
    {
        auto obj = this->known_object_at(x, y);
        if (obj == OBJECT::EMPTY)
            cout << "   |";
        else if (obj == OBJECT::ROBOT)
        {
            cout << "R";
            Coord coord(x, y);
            int robot_num = this->get_robot_num_at(coord);
            if (robot_num > 1)
            {
                cout << "S" << robot_num;
            }
            else
            {
                for (auto it = robots.begin(); it != robots.end(); ++it)
                {
                    if ((*it)->get_coord() == coord)
                    {
                        cout << to_string((*it)->type)[0] << (*it)->id;
                        break;
                    }
                }
            }
            cout << "|";
        }
        else if (obj == OBJECT::TASK)
        {
            for (auto it = tasks.begin(); it != tasks.end(); ++it)
            {
                if ((*it)->coord == Coord(x, y) && !(*it)->is_done())
                {
                    cout << "T" << setfill('0') << setw(2) << (*it)->id << setw(0) << setfill(' ') << '|';
                    break;
                }
            }
        }
        else if (obj == OBJECT::ROBOT_AND_TASK)
        {
            vector<shared_ptr<ROBOT>> result;
            Coord coord(x, y);
            cout << "T";
            int robot_num = this->get_robot_num_at(coord);
            if (robot_num > 1)
            {
                cout << "S" << robot_num;
            }
            else
            {
                for (auto it = robots.begin(); it != robots.end(); ++it)
                {
                    if ((*it)->get_coord() == coord)
                    {
                        cout << to_string((*it)->type)[0] << (*it)->id;
                        break;
                    }
                }
            }
            cout << "|";
        }
        else
            cout << to_string(obj).substr(0, 3) << "|";
    };
    print_base(f);
}
void MAP::print_robot_summary() const
{
    cout << "- Robot summary" << endl;
    cout << right << setw(2) << "ID"
         << "  "
         << left << setw(11) << "Type"
         << "  "
         << left << setw(8) << "Coord"
         << "  "
         << right << setw(6) << "Energy"
         << "  "
         << left << setw(9) << "Status"
         << "  "
         << left << setw(11) << "TargetCoord"
         << "  "
         << right << setw(4) << "Task"
         << "  "
         << endl;

    for (auto robot : robots)
    {
        cout << right << setw(2) << robot->id << "  "
             << left << setw(11) << robot->type << "  "
             << setw(8) << robot->coord << "  "
             << right << setw(6) << robot->energy << "  "
             << left << setw(9) << robot->status << "  "
             << left << setw(11) << robot->target_coord << "  "
             << right << setw(4) << ((robot->assigned_task.expired()) ? "No" : to_string(robot->assigned_task.lock()->id)) << "  "
             << endl;
    }
    cout << endl;
}
void MAP::print_task_summary() const
{
    cout << "- Task summary" << endl;
    cout << "Max Task : " << num_total_task << ", Task created : " << tasks.size() << ", Active task: " << active_tasks.size() << ", Completed task : " << completed_task_num << endl;
    cout << left << setw(4) << "ID" << setw(10) << "Location" << left << setw(7) << "Found" << setw(7) << "Done" << setw(0) << "Assigned  " << setw(0);
    for (int i = 1; i < ROBOT::NUM_ROBOT_TYPE; ++i)
    {
        cout << ROBOT::TYPE(i) << "  ";
    }
    cout << endl;

    for (auto &task : tasks)
    {
        cout << right << setw(2) << task->id
             << setw(10) << task->coord
             << setw(7) << bool_str(task->done || bool(known_object_at(task->coord) & OBJECT::TASK))
             << setw(7) << bool_str(task->done) << "  "
             << right << setw(8) << ((task->assigned_robot.expired()) ? "No" : to_string(task->assigned_robot.lock()->id));
        for (int i = 1; i < ROBOT::NUM_ROBOT_TYPE; ++i)
        {
            cout << setw(to_string(ROBOT::TYPE(i)).size() + 2) << task->task_cost[i];
        }
        cout << endl;
    }
    cout << endl;
}
// MAP end