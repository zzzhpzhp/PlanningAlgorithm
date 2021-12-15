#pragma once

#include <iostream>
#include <chrono>
#include <thread>
#include <stack>
#include <deque>
#include <queue>
#include <unordered_map>

#include <boost/bind.hpp>
#include <utility>

#include "environment_interface.h"
#include "algorithm_interface.h"

namespace algorithm
{
    class Bcd : public AlgorithmInterface
    {
    public:
        Bcd(environment::EnvironmentInterfacePtr &env, std::string name)
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
        environment::Path path_;
    };
}