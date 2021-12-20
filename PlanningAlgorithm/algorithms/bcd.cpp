#include "bcd.h"

namespace algorithm
{
    void Bcd::initialize(environment::EnvironmentInterfacePtr &env, std::string name)
    {
        env_ptr_ = env;
        name_ = name;

        initialized_ = true;
    }

    void Bcd::setGoal(int x, int y)
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

    void Bcd::setStart(int x, int y)
    {
        int tx, ty;
        if (!env_ptr_->displayXY2PlanningXY(x, y, tx, ty))
        {
            throw std::runtime_error("Coordinate transform failed.");
        }

        start_x_ = tx;
        start_y_ = ty;
        std::cout << "Set start to [" << start_x_ << ", " << start_y_ << "]" << std::endl;
    }

    bool Bcd::planning()
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

        std::unordered_map<int, std::unordered_map<int, bool>> visited;
        environment::PathNode pn{};
        pn.g = 255;
        pn.a = 255;
        path_.clear();

        auto cur_x = start_x_;
        auto cur_y = start_y_;

        auto feasible = [&](int x, int y)->bool
        {
            if (x < 0 || x >= env_ptr_->getGridXSizeInCells() ||
                y < 0 || y >= env_ptr_->getGridYSizeInCells())
            {
                return false;
            }

            auto side_val = env_ptr_->getGridValue(x, y);
            return !(visited[x][y] || side_val == 0);
        };

        auto save_path = [&](int x, int y)
        {
            pn.x = x;
            pn.y = y;
            path_.emplace_back(pn);
        };

        auto visit = [&](int x, int y)
        {
            visited[x][y] = true;
            cur_x = x;
            cur_y = y;
        };

        enum Dir {X_POSITIVE, X_NEGATIVE};
        Dir dir = Dir::X_POSITIVE;

        std::function<bool()> bcd = [&]()->bool
        {
            int sx, sy;

            while (is_running_.load())
            {
                std::this_thread::sleep_for(std::chrono::microseconds((int)(env_ptr_->getAlgorithmRunningDelayTime() * 1e6)));
                // 標記已訪問區域
                if (cur_x != start_x_ || cur_y != start_y_)
                {
                    env_ptr_->setIntGridValByPlanXY(cur_x, cur_y, 100, 100, 100);
                }
                // 存儲路徑
                save_path(cur_x, cur_y);

                if (dir == Dir::X_POSITIVE)
                {
                    // 尝试往x轴正方向移动
                    sx = cur_x + 1;
                    sy = cur_y;
                    if (feasible(sx, sy))
                    {
                        visit(sx, sy);
                        continue;
                    }
                    else
                    {
                        dir = Dir::X_NEGATIVE;
                    }
                }
                else if (dir == Dir::X_NEGATIVE)
                {
                    // 尝试往x轴负方向移动
                    sx = cur_x - 1;
                    sy = cur_y;

                    if (feasible(sx, sy))
                    {
                        visit(sx, sy);
                        continue;
                    }
                    else
                    {
                        dir = Dir::X_POSITIVE;
                    }
                }

                // 尝试往y轴负方向移动
                sx = cur_x;
                sy = cur_y - 1;

                if (feasible(sx, sy))
                {
                    visit(sx, sy);
                    continue;
                }
                else
                {
                    break;
                }
            }

            return true;
        };

        auto result = bcd();
        return result;
    }

    environment::Path &Bcd::getPath()
    {
        return path_;
    }
}