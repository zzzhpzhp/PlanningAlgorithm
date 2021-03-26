#include "dfs.h"

namespace planner
{
    void Dfs::initialize(EnvironmentInterfacePtr &env)
    {
        env_ptr_ = env;

        side_points_.emplace_back(boost::bind(&Dfs::_get_right, this, _1, _2, _3, _4));
//        side_points_.emplace_back(boost::bind(&Dfs::_get_higher_right, this, _1, _2, _3, _4));
        side_points_.emplace_back(boost::bind(&Dfs::_get_middle_higher, this, _1, _2, _3, _4));
//        side_points_.emplace_back(boost::bind(&Dfs::_get_higher_left, this, _1, _2, _3, _4));
        side_points_.emplace_back(boost::bind(&Dfs::_get_left, this, _1, _2, _3, _4));
//        side_points_.emplace_back(boost::bind(&Dfs::_get_lower_left, this, _1, _2, _3, _4));
        side_points_.emplace_back(boost::bind(&Dfs::_get_middle_lower, this, _1, _2, _3, _4));
//        side_points_.emplace_back(boost::bind(&Dfs::_get_lower_right, this, _1, _2, _3, _4));

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


//        struct node
//        {
//            int x, y;
//            int id;
//        };
//        std::stack<node> nodes_stack;
//        node start;
//        start.x = start_x_;
//        start.y = start_y_;
//        start.id = start.y * env_ptr_->getGridXSizeInCells() + start.x;
        boost::unordered_map<int,boost::unordered_map<int, bool>> checked;
        std::function<bool(int, int)> dfs = [&](int x, int y)->bool
        {
//            auto &cur = nodes_stack.top();
//            nodes_stack.pop();
            if (x == goal_x_ && y == goal_y_)
            {
                return true;
            }

            checked[x][y] = true;
            int side_x, side_y;
            for (auto &side_node : side_points_)
            {
                if (!side_node(x, y, side_x, side_y))
                {
                    continue;
                }
                if (checked[side_x][side_y])
                {
                    continue;
                }
//                std::cout << side_x << " " << side_y << "\n";
                env_ptr_->setGridValue(side_x, side_y, 0);
                return dfs(side_x, side_y);
            }

            return false;
        };

        return dfs(start_x_, start_y_);
    }
}