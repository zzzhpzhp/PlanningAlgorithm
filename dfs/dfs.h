#ifndef DFS_DFS_H
#define DFS_DFS_H

#include <iostream>
#include <chrono>
#include <thread>
#include <stack>

#include <boost/unordered_map.hpp>
#include <boost/bind.hpp>

#include "environment_interface.h"
#include "algorithm_interface.h"

namespace planner
{
    class Dfs : public AlgorithmInterface
    {
    public:
        void
        initialize(EnvironmentInterfacePtr &env) override;

        void
        setGoal(int x, int y) override;

        void
        setStart(int x, int y) override;

        bool
        planning() override;

    private:
        bool initialized_{false};

        int start_x_{0}, start_y_{0};
        int goal_x_{0}, goal_y_{0};

        EnvironmentInterfacePtr env_ptr_;
        std::vector<std::function<bool(EnvironmentInterfacePtr&, int, int, int&, int&)>> side_points_;
    };
}

#endif //DFS_DFS_H
