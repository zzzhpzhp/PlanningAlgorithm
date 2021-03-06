#pragma once

#include <iostream>
#include <chrono>
#include <thread>
#include <stack>
#include <unordered_map>

#include <boost/bind.hpp>

#include "environment_interface.h"
#include "algorithm_interface.h"

namespace algorithm
{
    class BcdWithFootprint : public AlgorithmInterface
    {
    public:

        BcdWithFootprint(environment::EnvironmentInterfacePtr &env, std::string name)
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

        int robot_radius_{8};
        int start_x_{0}, start_y_{0};
        int goal_x_{0}, goal_y_{0};

        // 规划衔接路径时，地图代价的比重，越大规划的路径会越远离障碍物
        float map_cost_scale_{0.0f};

        environment::EnvironmentInterfacePtr env_ptr_;
        std::vector<std::function<bool(environment::EnvironmentInterfacePtr&, int, int, int&, int&)>> side_points_, dijkstra_side_points_;
        environment::Path path_;
        environment::Path last_cover_path_;
        environment::Path last_bridge_path_;

        using VisitedTable = std::unordered_map<int, std::unordered_map<int, bool>>;
        std::unordered_map<int, std::unordered_map<int, bool>> visited_, cleaned_;

        struct Node
        {
            int x{0}, y{0};
            float dist{0.0f};
            Node *parent_node{nullptr};
            bool in_open_list = false;
            bool in_close_list = false;

        };
        struct NodeCmp
        {
            bool operator()(const Node* a, const Node* n) const
            {
                return a->dist > n->dist;
            };
        };
        std::vector<Node> nodes_;

        std::function<bool(int, int)> reach_judge_;
        std::atomic_bool search_leap_{false};

        bool _position_validation(int x, int y);
        void _mark_up_down_covered(int x, int y);
        void _mark_footprint_covered(int x, int y);
        void _mark_footprint_covered(const environment::Path& path);
        bool _goal_reached(int x, int y);
        bool _dijkstra(int start_x, int start_y, int &goal_x, int &goal_y,
                       VisitedTable& visited, std::vector<environment::PathNode> &path);
    };
}