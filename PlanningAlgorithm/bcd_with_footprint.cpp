#include "bcd_with_footprint.h"

namespace algorithm
{
    void BcdWithFootprint::initialize(environment::EnvironmentInterfacePtr &env)
    {
        env_ptr_ = env;

        side_points_.emplace_back(boost::bind(&BcdWithFootprint::_get_right, this, _1, _2, _3, _4, _5));
        side_points_.emplace_back(boost::bind(&BcdWithFootprint::_get_left, this, _1, _2, _3, _4, _5));
        side_points_.emplace_back(boost::bind(&BcdWithFootprint::_get_middle_higher, this, _1, _2, _3, _4, _5));
        side_points_.emplace_back(boost::bind(&BcdWithFootprint::_get_middle_lower, this, _1, _2, _3, _4, _5));

//        side_points_.emplace_back(boost::bind(&BcdWithFootprint::_get_higher_right, this, _1, _2, _3, _4, _5));
//        side_points_.emplace_back(boost::bind(&BcdWithFootprint::_get_higher_left, this, _1, _2, _3, _4, _5));
//        side_points_.emplace_back(boost::bind(&BcdWithFootprint::_get_lower_left, this, _1, _2, _3, _4, _5));
//        side_points_.emplace_back(boost::bind(&BcdWithFootprint::_get_lower_right, this, _1, _2, _3, _4, _5));

        for (int i = -2; i <= 2; i++)
        {
//            for (int j = -robot_radius_; j < robot_radius_; j++)
//            {
                if (std::hypot(0, i) < robot_radius_)
                {
                    catched_area_.emplace_back(std::tuple<int, int>(0, i));
                }
//            }
        }
        initialized_ = true;
    }

    void BcdWithFootprint::setGoal(int x, int y)
    {
        int tx, ty;
        if (!env_ptr_->displayXY2PlanningXY(x, y, tx, ty))
        {
            throw std::runtime_error("Coordinate transform failed.");
        }
        goal_x_ = tx;
        goal_y_ = ty;
        std::cout << "Set goal to ["<< goal_x_ << ", " << goal_y_ << "]" << std::endl;
    }

    void BcdWithFootprint::setStart(int x, int y)
    {
        int tx, ty;
        if (!env_ptr_->displayXY2PlanningXY(x, y, tx, ty))
        {
            throw std::runtime_error("Coordinate transform failed.");
        }

        start_x_ = tx;
        start_y_ = ty;
        std::cout << "Set start to ["<< start_x_ << ", " << start_y_ << "]" << std::endl;
    }

    bool BcdWithFootprint::planning()
    {
        if (!initialized_)
        {
            std::cerr << "Should initialize first." << std::endl;
            return false;
        }

        if (!env_ptr_->insideGrid(goal_x_, goal_y_) || !env_ptr_->insideGrid(start_x_, start_y_))
        {
            std::cerr << "Start or Goal pose out of bound." << std::endl;
            return false;
        }

        std::cout << "Start pose: [" << start_x_ << ", " << start_y_ << "]" << std::endl;
        std::cout << "Goal pose: [" << goal_x_ << ", " << goal_y_ << "]" << std::endl;

        std::stack<std::pair<int,int>> node_stack;
        environment::PathNode pn{};
        pn.g = 255;
        pn.a = 255;
        path_.clear();
        visited_.clear();

        std::function<bool(int, int)> BcdWithFootprint = [&](int x, int y)->bool
        {
            std::this_thread::sleep_for(std::chrono::microseconds(1000));

            if (x == goal_x_ && y == goal_y_)
            {
                return true;
            }
            if (x != start_x_ || y != start_y_)
            {
                env_ptr_->setIntGridValByPlanXY(x, y, 100, 100, 100);
            }
            int side_x, side_y;
            uint8_t side_val;
            for (auto &side_node : side_points_)
            {
                if (!side_node(env_ptr_, x, y, side_x, side_y))
                {
                    continue;
                }
//                side_val = env_ptr_->getGridValue(side_x, side_y);
                if (visited_[side_x][side_y])
                {
                    continue;
                }

                if (_position_validation(side_x, side_y))
                {
                    visited_[side_x][side_y] = true;
                    pn.x = side_x;
                    pn.y = side_y;
                    path_.emplace_back(pn);
                    _mark_visited(side_x, side_y);
                }

                if (BcdWithFootprint(side_x, side_y))
                {
                    return true;
                }
            }

            return false;
        };

        auto result = BcdWithFootprint(start_x_, start_y_);
        return result;
    }

    environment::Path& BcdWithFootprint::getPath()
    {
        return path_;
    }
}