#include "dfs.h"

namespace aff
{
    void Dfs::initialize(EnvironmentInterfacePtr &env)
    {
        env_ptr_ = env;

        side_points_.emplace_back(boost::bind(&Dfs::_get_right, this, _1, _2, _3, _4, _5));
//        side_points_.emplace_back(boost::bind(&Dfs::_get_higher_right, this, _1, _2, _3, _4, _5));
        side_points_.emplace_back(boost::bind(&Dfs::_get_middle_higher, this, _1, _2, _3, _4, _5));
//        side_points_.emplace_back(boost::bind(&Dfs::_get_higher_left, this, _1, _2, _3, _4, _5));
        side_points_.emplace_back(boost::bind(&Dfs::_get_left, this, _1, _2, _3, _4, _5));
//        side_points_.emplace_back(boost::bind(&Dfs::_get_lower_left, this, _1, _2, _3, _4, _5));
        side_points_.emplace_back(boost::bind(&Dfs::_get_middle_lower, this, _1, _2, _3, _4, _5));
//        side_points_.emplace_back(boost::bind(&Dfs::_get_lower_right, this, _1, _2, _3, _4, _5));

        initialized_ = true;
    }

    void Dfs::setGoal(int x, int y)
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

    void Dfs::setStart(int x, int y)
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

    bool Dfs::planning()
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


        std::unordered_map<int, std::unordered_map<int, bool>> visited;
        std::function<bool(int, int)> dfs = [&](int x, int y)->bool
        {
            if (x == goal_x_ && y == goal_y_)
            {
                return true;
            }
            if (x != start_x_ || y != start_y_)
            {
                env_ptr_->setIntGridValByPlanXY(x, y, 100, 100, 100);
                env_ptr_->display();
            }

            visited[x][y] = true;
            int side_x, side_y;
            uint8_t side_val;
            for (auto &side_node : side_points_)
            {
                if (!side_node(env_ptr_, x, y, side_x, side_y))
                {
                    continue;
                }
                side_val = env_ptr_->getGridValue(side_x, side_y);
                if (visited[side_x][side_y] || side_val == 0)
                {
                    continue;
                }

                if (dfs(side_x, side_y))
                {
                    return true;
                }
            }

            return false;
        };

        auto result = dfs(start_x_, start_y_);
        return result;
    }
}