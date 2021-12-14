#ifndef AFF_DIJKSTRA_H
#define AFF_DIJKSTRA_H

#include <stack>
#include <deque>
#include <queue>
#include <chrono>
#include <thread>
#include <iostream>
#include <unordered_map>

#include <boost/bind.hpp>

#include "algorithm_interface.h"
#include "environment_interface.h"

namespace algorithm
{
    class Dijkstra : public AlgorithmInterface
    {
    public:
        void
        initialize(environment::EnvironmentInterfacePtr &env) override;

        void
        setGoal(int x, int y) override;

        void
        setStart(int x, int y) override;

        bool
        planning() override;

        environment::Path&
        getPath() override;

    private:
        bool initialized_{false};

        int start_x_{0}, start_y_{0};
        int goal_x_{0}, goal_y_{0};

        environment::EnvironmentInterfacePtr env_ptr_;
        std::vector<std::function<bool(environment::EnvironmentInterfacePtr&, int, int, int&, int&)>> side_points_;
        environment::Path path_;

        struct Node
        {
            int id;
            int x, y;
            int dist = 0;
            Node *parent_node;

            bool is_obstacle = false;
            bool in_open_list = false;
            bool in_close_list = false;

            bool operator()(const Node* a, const Node* n) const
            {
                return a->dist > n->dist;
            };
        };
    };
}

#endif //AFF_DIJKSTRA_H
