#ifndef DFS_DFS_H
#define DFS_DFS_H

#include <iostream>
#include <chrono>
#include <thread>
#include <stack>

#include <boost/unordered_map.hpp>
#include <boost/bind.hpp>

#include "environment_interface.h"

namespace planner
{
#define DEBUG_PRINT { std::cout << __FUNCTION__ << " " << __LINE__ << std::endl;}
    class Dfs
    {
    public:
        void
        initialize(EnvironmentInterfacePtr &env);

        void
        setGoal(int x, int y);

        void
        setStart(int x, int y);

        bool
        planning();

    private:
        bool initialized_{false};

        int start_x_{0}, start_y_{0};
        int goal_x_{0}, goal_y_{0};

        EnvironmentInterfacePtr env_ptr_;
        std::vector<std::function<bool(int, int, int&, int&)>> side_points_;

        inline bool
        _get_middle_higher(int x, int y, int &sx, int &sy)
        {
            sx = x;
            sy = y - 1;
            if (env_ptr_->insideGrid(sx, sy))
            {
                return true;
            }
            return false;
        }
        inline bool
        _get_middle_lower(int x, int y, int &sx, int &sy)
        {
            sx = x;
            sy = y + 1;
            if (env_ptr_->insideGrid(sx, sy))
            {
                return true;
            }
            return false;
        }
        inline bool
        _get_left(int x, int y, int &sx, int &sy)
        {
            sx = x - 1;
            sy = y;
            if (env_ptr_->insideGrid(sx, sy))
            {
                return true;
            }
            return false;
        }
        inline bool
        _get_right(int x, int y, int &sx, int &sy)
        {
            sx = x + 1;
            sy = y;
            if (env_ptr_->insideGrid(sx, sy))
            {
                return true;
            }
            return false;
        }
        inline bool
        _get_higher_left(int x, int y, int &sx, int &sy)
        {
            sx = x - 1;
            sy = y - 1;
            if (env_ptr_->insideGrid(sx, sy))
            {
                return true;
            }
            return false;
        }
        inline bool
        _get_higher_right(int x, int y, int &sx, int &sy)
        {
            sx = x + 1;
            sy = y - 1;
            if (env_ptr_->insideGrid(sx, sy))
            {
                return true;
            }
            return false;
        }
        inline bool
        _get_lower_left(int x, int y, int &sx, int &sy)
        {
            sx = x - 1;
            sy = y + 1;
            if (env_ptr_->insideGrid(sx, sy))
            {
                return true;
            }
            return false;
        }
        inline bool
        _get_lower_right(int x, int y, int &sx, int &sy)
        {
            sx = x + 1;
            sy = y + 1;
            if (env_ptr_->insideGrid(sx, sy))
            {
                return true;
            }
            return false;
        }

    };
}

#endif //DFS_DFS_H
