#ifndef AFF_DFS_H
#define AFF_DFS_H

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
    class Dfs : public AlgorithmInterface
    {
    public:
        Dfs(environment::EnvironmentInterfacePtr &env, std::string name)
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
    };
}

#endif //AFF_DFS_H
