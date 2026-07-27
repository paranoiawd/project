#ifndef SCHEDULER_H_
#define SCHEDULER_H_

#include "simulator.h"
#include <memory>

class Scheduler
{
public:
    Scheduler();
    ~Scheduler();

    void on_info_updated(const set<Coord> &observed_coords,
                         const set<Coord> &updated_coords,
                         const vector<vector<vector<int>>> &known_cost_map,
                         const vector<vector<OBJECT>> &known_object_map,
                         const vector<shared_ptr<TASK>> &active_tasks,
                         const vector<shared_ptr<ROBOT>> &robots);

    bool on_task_reached(const set<Coord> &observed_coords,
                         const set<Coord> &updated_coords,
                         const vector<vector<vector<int>>> &known_cost_map,
                         const vector<vector<OBJECT>> &known_object_map,
                         const vector<shared_ptr<TASK>> &active_tasks,
                         const vector<shared_ptr<ROBOT>> &robots,
                         const ROBOT &robot,
                         const TASK &task);

    ROBOT::ACTION idle_action(const set<Coord> &observed_coords,
                              const set<Coord> &updated_coords,
                              const vector<vector<vector<int>>> &known_cost_map,
                              const vector<vector<OBJECT>> &known_object_map,
                              const vector<shared_ptr<TASK>> &active_tasks,
                              const vector<shared_ptr<ROBOT>> &robots,
                              const ROBOT &robot);

private:
    // All planning state and helpers live in schedular.cpp (see Scheduler::State).
    struct State;
    unique_ptr<State> s_;
};
#endif // SCHEDULER_H_
