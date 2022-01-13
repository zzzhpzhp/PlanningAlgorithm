#pragma once

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
    class JPS : public AlgorithmInterface
    {
    public:
        JPS(environment::EnvironmentInterfacePtr &env, std::string name)
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

        int node_cnt_{0};
        int start_x_{0}, start_y_{0};
        int goal_x_{0}, goal_y_{0};

        enum SIDE_POSE {ML, MH, L, R, HL, HR, LL, LR, INVALID};
        environment::EnvironmentInterfacePtr env_ptr_;

        using SIDE_ARCHIVE = bool(environment::EnvironmentInterfacePtr&, int, int, int&, int&);
        using SIDE_ARCHIVE_POINTER = std::function<SIDE_ARCHIVE>;
        std::vector<SIDE_ARCHIVE_POINTER> side_points_;

        environment::Path path_;

        struct Node
        {
            int x, y;
            double dist = 0.0f;
            // 启发值
            float h{0.0f};
            Node *parent_node{nullptr};

            bool is_obstacle = false;
            bool in_open_list = false;
            bool in_close_list = false;
            bool visited = false;
        };

#define DISTANCE_SCALE 0.1
        struct NodeCmp
        {

            bool operator()(const Node* a, const Node* n) const
            {
                return a->dist * DISTANCE_SCALE + a->h > n->dist * DISTANCE_SCALE + n->h;
            };
        };

        float calc_h_(Node *n)
        {
            return (float)(abs(n->x - goal_x_)) + abs((n->y - goal_y_));
        }

        bool _is_obstacle(int cost)
        {
            return (cost >= 0 && cost <= environment::INSCRIBED_INFLATED_OBSTACLE);
        }

        bool _have_forced_neighbor(int cx, int cy, SIDE_POSE pos);

        SIDE_POSE _get_pose(int cx, int cy, int px, int py);
    };
}
