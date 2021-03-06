#include "dfs.h"

namespace algorithm
{
    void Dfs::initialize(environment::EnvironmentInterfacePtr &env, std::string name)
    {
        env_ptr_ = env;
        name_ = name;

        side_points_.emplace_back(boost::bind(&Dfs::_get_right, this, _1, _2, _3, _4, _5, 1));
        side_points_.emplace_back(boost::bind(&Dfs::_get_left, this, _1, _2, _3, _4, _5, 1));
        side_points_.emplace_back(boost::bind(&Dfs::_get_middle_higher, this, _1, _2, _3, _4, _5, 1));
        side_points_.emplace_back(boost::bind(&Dfs::_get_middle_lower, this, _1, _2, _3, _4, _5, 1));

//        side_points_.emplace_back(boost::bind(&Dfs::_get_higher_right, this, _1, _2, _3, _4, _5, 1));
//        side_points_.emplace_back(boost::bind(&Dfs::_get_higher_left, this, _1, _2, _3, _4, _5, 1));
//        side_points_.emplace_back(boost::bind(&Dfs::_get_lower_left, this, _1, _2, _3, _4, _5, 1));
//        side_points_.emplace_back(boost::bind(&Dfs::_get_lower_right, this, _1, _2, _3, _4, _5, 1));

        initialized_ = true;
    }

    void Dfs::setGoal(int x, int y)
    {
        int tx, ty;
        if (!env_ptr_->toGridAndInsideGrid(x, y, tx, ty))
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
        if (!env_ptr_->toGridAndInsideGrid(x, y, tx, ty))
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
        environment::PathNode pn{};
        pn.g = 255;
        pn.a = 255;
        path_.clear();

        std::unordered_map<int, std::unordered_map<int, bool>> visited;
        std::function<bool(int, int)> dfs = [&](int x, int y)->bool
        {
            if (!is_running_.load())
            {
                return false;
            }
            std::this_thread::sleep_for(std::chrono::microseconds((int)(env_ptr_->getAlgorithmRunningDelayTime() * 1e6)));
            pn.x = x;
            pn.y = y;
            path_.emplace_back(pn);

//            if (x == goal_x_ && y == goal_y_)
//            {
//                return true;
//            }
            if (x != start_x_ || y != start_y_)
            {
                env_ptr_->setIntGridValueByGridXY(x, y, 100, 100, 100);
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
                if (visited[side_x][side_y] || side_val <= environment::INSCRIBED_INFLATED_OBSTACLE)
                {
                    continue;
                }

                dfs(side_x, side_y);
            }

            return true;
        };

        auto result = dfs(start_x_, start_y_);
        return result;
    }

    environment::Path& Dfs::getPath()
    {
        return path_;
    }
}