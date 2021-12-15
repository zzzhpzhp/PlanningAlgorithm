#ifndef AFF_BCD_WITH_DIJKSTRA_H
#define AFF_BCD_WITH_DIJKSTRA_H

#include <iostream>
#include <chrono>
#include <thread>
#include <stack>
#include <deque>
#include <queue>
#include <unordered_map>

#include <boost/bind.hpp>

#include "environment_interface.h"
#include "algorithm_interface.h"

namespace algorithm
{
    class BcdWidthDijkstra : public AlgorithmInterface
    {
    public:
        BcdWidthDijkstra(environment::EnvironmentInterfacePtr &env, std::string name)
        {
            initialize(env, std::move(name));
        }

        void
        initialize(environment::EnvironmentInterfacePtr &env, std::string name={}) override;

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
        using VisitedTable = std::unordered_map<int, std::unordered_map<int, bool>>;

        struct Node
        {
            int x, y;
            int id;
            int dist;
            Node *parent_node;
            bool in_open_list = false;
            bool in_close_list = false;
            bool is_obstacle = false;

            bool operator()(const Node* a, const Node* n) const
            {
                return a->dist > n->dist;
            };
        };
        std::vector<Node> nodes_;
        VisitedTable visited_;

        bool _dijkstra(int start_x, int start_y, int &goal_x, int &goal_y,
            VisitedTable& visited, std::vector<environment::PathNode> &path);
    };
}

#endif //AFF_BCD_WITH_DIJKSTRA_H
