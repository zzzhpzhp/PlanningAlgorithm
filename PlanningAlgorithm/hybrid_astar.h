#ifndef AFF_HYBRID_ASTAR_H
#define AFF_HYBRID_ASTAR_H

#include <iostream>
#include <chrono>
#include <thread>
#include <stack>
#include <deque>
#include <queue>
#include <tuple>
#include <unordered_map>

#include <boost/bind.hpp>

#include "environment_interface.h"
#include "algorithm_interface.h"

namespace algorithm
{
    class HybirdAstar : public AlgorithmInterface
    {
    public:
        HybirdAstar(environment::EnvironmentInterfacePtr &env, std::string name)
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

        double min_turn_radius_ = 1.0;
        double turn_cost_scale_ = 1.0;
        double map_cost_scale_ = 1.0;
        double length_cost_scale_ = 1.0;

        int start_x_{0}, start_y_{0};
        int goal_x_{0}, goal_y_{0};

        environment::EnvironmentInterfacePtr env_ptr_;
        std::vector<std::function<bool(environment::EnvironmentInterfacePtr&, int, int, int&, int&)>> side_points_;
        environment::Path path_;

        struct Node
        {
            int x, y;
            int id;
            int dist = 0;
            Node *parent_node;

            bool in_open_list = false;
            bool in_close_list = false;
            bool is_obstacle = false;

            bool operator()(const Node* a, const Node* n) const
            {
                return a->dist > n->dist;
            };
        };

        std::tuple<double, double>
        getForwardLeftPose(double x, double y);

        std::tuple<double, double>
        getForwardRightPose(double x, double y);
    };
}


#endif //AFF_HYBRID_ASTAR_H
