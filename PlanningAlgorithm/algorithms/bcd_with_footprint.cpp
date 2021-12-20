#include "bcd_with_footprint.h"

namespace algorithm
{
    void BcdWithFootprint::initialize(environment::EnvironmentInterfacePtr &env, std::string name)
    {
        env_ptr_ = env;
        name_ = name;
        robot_radius_ = env_ptr_->getRobotRadius();
        side_points_.emplace_back(boost::bind(&BcdWithFootprint::_get_right, this, _1, _2, _3, _4, _5, 1));
        side_points_.emplace_back(boost::bind(&BcdWithFootprint::_get_left, this, _1, _2, _3, _4, _5, 1));
        side_points_.emplace_back(boost::bind(&BcdWithFootprint::_get_middle_higher, this, _1, _2, _3, _4, _5, robot_radius_));
        side_points_.emplace_back(boost::bind(&BcdWithFootprint::_get_middle_lower, this, _1, _2, _3, _4, _5, robot_radius_));

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
        int limiting_index_l{0}, limiting_index_h{0};
        pn.g = 255;
        pn.a = 255;
        path_.clear();
        visited_.clear();
        cleaned_.clear();
        in_path_.clear();

        if (_position_validation(start_x_, start_y_, limiting_index_l, limiting_index_h))
        {
            _mark_cleaned(start_x_, start_y_, limiting_index_l, limiting_index_h);
            env_ptr_->setIntGridValByPlanXY(start_x_, start_y_, 100, 100, 100);
            in_path_[start_x_][start_y_] = true;
        }
        visited_[start_x_][start_y_] = true;
        pn.x = start_x_;
        pn.y = start_y_;
        path_.emplace_back(pn);

        std::function<bool(int, int)> BcdWithFootprint = [&](int x, int y)->bool
        {
            if (!is_running_)
            {
                return false;
            }
            std::this_thread::sleep_for(std::chrono::microseconds((int)(env_ptr_->getAlgorithmRunningDelayTime() * 1e6)));
            int side_x, side_y;
            uint8_t side_val;
            bool valid = false;
            for (auto &side_node : side_points_)
            {
                if (!side_node(env_ptr_, x, y, side_x, side_y))
                {
                    continue;
                }
                side_val = env_ptr_->getGridValue(side_x, side_y);
                if (side_val == 0 || visited_[side_x][side_y])
                {
                    continue;
                }
                visited_[side_x][side_y] = true;
                if (_position_validation(side_x, side_y, limiting_index_l, limiting_index_h))
                {
                    valid = true;
                    _mark_cleaned(side_x, side_y, limiting_index_l, limiting_index_h);
                    env_ptr_->setIntGridValByPlanXY(side_x, side_y, 100, 100, 100);
                    pn.x = side_x;
                    pn.y = side_y;
                    path_.emplace_back(pn);
                    in_path_[side_x][side_y] = true;
                    if (!BcdWithFootprint(side_x, side_y))
                    {
                        return false;
                    }
                }
            }
            if (!valid)
            {
                // Dead point
                env_ptr_->setIntGridValByPlanXY(x, y, 100, 0, 0);
            }
            return true;
        };

        return BcdWithFootprint(start_x_, start_y_);;
    }

    environment::Path& BcdWithFootprint::getPath()
    {
        return path_;
    }

    bool BcdWithFootprint::_position_validation(int x, int y, int &limiting_index_l, int &limiting_index_h)
    {
        limiting_index_l = limiting_index_h = 0;
        bool res{false};
        for (int i = 0; i > -robot_radius_; i--)
        {
            auto tx = x;
            auto ty = i + y;
            if (!env_ptr_->insideGrid(tx, ty))
            {
                break;
            }

            auto val = env_ptr_->getGridValue(tx, ty);
            if (cleaned_[tx][ty] || val == 0)
            {
                break;
            }
            res = true;
            limiting_index_l = i;
        }

        // 以下部分的判断仅仅是为了得到另一边的覆盖情况，以便完善显示信息，不影响此点的可通行判断
        for (int i = 1; i < robot_radius_; i++)
        {
            auto tx = x;
            auto ty = i + y;
            if (!env_ptr_->insideGrid(tx, ty))
            {
                break;
            }

            auto val = env_ptr_->getGridValue(tx, ty);
            if (val == 0)
            {
                break;
            }
            limiting_index_h = i;
        }
        return res;
    }

    void BcdWithFootprint::_mark_cleaned(int x, int y, int limiting_l, int limiting_h)
    {
        for (int i = limiting_l; i <= 0; i++)
        {
            auto tx = x;
            auto ty = i + y;
            if (!env_ptr_->insideGrid(tx, ty))
            {
                continue;
            }
            cleaned_[tx][ty] = true;
            if (!in_path_[tx][ty])
            {
                env_ptr_->setIntGridValByPlanXY(tx, ty, 150, 150, 150);
            }
        }

        for (int i = 1; i <= limiting_h; i++)
        {
            auto tx = x;
            auto ty = i + y;
            if (!env_ptr_->insideGrid(tx, ty))
            {
                continue;
            }
            if (!in_path_[tx][ty])
            {
                env_ptr_->setIntGridValByPlanXY(tx, ty, 150, 150, 150);
            }
        }
    }
}