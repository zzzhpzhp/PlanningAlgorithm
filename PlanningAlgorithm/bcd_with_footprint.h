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

        int robot_radius_{3};
        int start_x_{0}, start_y_{0};
        int goal_x_{0}, goal_y_{0};

        environment::EnvironmentInterfacePtr env_ptr_;
        std::vector<std::function<bool(environment::EnvironmentInterfacePtr&, int, int, int&, int&)>> side_points_;
        environment::Path path_;
        std::vector<std::tuple<int, int>> catched_area_;

        std::unordered_map<int, std::unordered_map<int, bool>> visited_, cleaned_;

        bool _position_validation(int x, int y)
        {
            for (auto p : catched_area_)
            {
                auto tx = std::get<0>(p) + x;
                auto ty = std::get<1>(p) + y;
                if (!env_ptr_->insideGrid(tx, ty))
                {
                    continue;
                }

                if (cleaned_[tx][ty])
                {
                    return false;
                }

                auto val = env_ptr_->getGridValue(tx, ty);
                if (val == 0)
                {
                    return false;
                }
            }
            return true;
        }
        
        bool _position_validation(int x, int y)
        {
            for (auto p : catched_area_)
            {
                auto tx = std::get<0>(p) + x;
                auto ty = std::get<1>(p) + y;
                if (!env_ptr_->insideGrid(tx, ty))
                {
                    continue;
                }

                if (cleaned_[tx][ty])
                {
                    return false;
                }

                auto val = env_ptr_->getGridValue(tx, ty);
                if (val == 0)
                {
                    return false;
                }
            }
            return true;
        }

        void _mark_visited(int x, int y)
        {
            for (const auto &p : catched_area_)
            {
                auto tx = std::get<0>(p) + x;
                auto ty = std::get<1>(p) + y;
                std::cout << tx << " " << ty << std::endl;
                std::cout << "     " << std::get<0>(p) << " "<< std::get<1>(p) << std::endl;
                if (!env_ptr_->insideGrid(tx, ty))
                {
                    continue;
                }

                cleaned_[tx][ty] = true;
                env_ptr_->setIntGridValByPlanXY(tx, ty, 150, 150, 150);
            }
        }
    };
}