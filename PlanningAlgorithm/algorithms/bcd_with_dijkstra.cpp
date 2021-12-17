#include "bcd_with_dijkstra.h"
#include "utility.h"

namespace algorithm
{
    void BcdWidthDijkstra::initialize(environment::EnvironmentInterfacePtr &env, std::string name)
    {
        env_ptr_ = env;
        name_ = name;
        name_ = name;

        side_points_.emplace_back(boost::bind(&BcdWidthDijkstra::_get_right, this, _1, _2, _3, _4, _5, 1));
        side_points_.emplace_back(boost::bind(&BcdWidthDijkstra::_get_middle_higher, this, _1, _2, _3, _4, _5, 1));
        side_points_.emplace_back(boost::bind(&BcdWidthDijkstra::_get_left, this, _1, _2, _3, _4, _5, 1));
        side_points_.emplace_back(boost::bind(&BcdWidthDijkstra::_get_middle_lower, this, _1, _2, _3, _4, _5, 1));

//        side_points_.emplace_back(boost::bind(&BcdWidthDijkstra::_get_higher_right, this, _1, _2, _3, _4, _5, 1));
//        side_points_.emplace_back(boost::bind(&BcdWidthDijkstra::_get_higher_left, this, _1, _2, _3, _4, _5, 1));
//        side_points_.emplace_back(boost::bind(&BcdWidthDijkstra::_get_lower_left, this, _1, _2, _3, _4, _5, 1));
//        side_points_.emplace_back(boost::bind(&BcdWidthDijkstra::_get_lower_right, this, _1, _2, _3, _4, _5, 1));


        initialized_ = true;
    }

    void BcdWidthDijkstra::setGoal(int x, int y)
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

    void BcdWidthDijkstra::setStart(int x, int y)
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

    bool BcdWidthDijkstra::planning()
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

        path_.clear();
        visited_.clear();

        environment::PathNode pn{};
        pn.g = 255;
        pn.a = 255;
        auto feasible = [&](int x, int y)->bool
        {
            if (x < 0 || x >= env_ptr_->getGridXSizeInCells() ||
                y < 0 || y >= env_ptr_->getGridYSizeInCells())
            {
                return false;
            }

            auto side_val = env_ptr_->getGridValue(x, y);
            return !(visited_[x][y] || side_val == 0);
        };

        auto save_path = [&](int x, int y)
        {
            pn.x = x;
            pn.y = y;
            path_.emplace_back(pn);
        };

        auto cur_x = start_x_;
        auto cur_y = start_y_;

        auto visit = [&](int x, int y)
        {
            visited_[x][y] = true;
            cur_x = x;
            cur_y = y;
        };

        visited_[cur_x][cur_y] = true;

        enum Dir {X_POSITIVE, X_NEGATIVE};
        Dir dir = Dir::X_POSITIVE;

        std::function<bool()> bcd = [&]()->bool
        {
            int sx, sy;

            while (true)
            {
                std::this_thread::sleep_for(std::chrono::microseconds(500));
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
                    if (!_dijkstra(cur_x, cur_y, cur_x, cur_y, visited_, path_))
                    {
                        break;
                    }
                    visit(cur_x, cur_y);
                    continue;
                }
            }

            return true;
        };

        auto result = bcd();

        return result;
    }

    bool BcdWidthDijkstra::_dijkstra(int start_x, int start_y, int &goal_x, int &goal_y,
                   VisitedTable& visited, std::vector<environment::PathNode> &path)
    {
        auto size_x = env_ptr_->getGridXSizeInCells(), size_y = env_ptr_->getGridYSizeInCells();
        nodes_ = std::vector<Node>(size_x * size_y);
        std::priority_queue<Node*, std::vector<Node*>, Node> nodes_queue;
        int id_index = start_y * size_x + start_x;
        Node *cur_ = &nodes_[id_index];
        cur_->x = start_x;
        cur_->y = start_y;
        cur_->dist = 0;
        cur_->id = id_index;
        nodes_queue.push(cur_);
        environment::PathNode pn{};
        pn.r = 255;
        pn.a = 255;
        bool find = false;
        int side_to_cur_cost{};
        int through_cur_cost{};
        int side_x, side_y;
        uint8_t side_val;
        Node *side;

        while (!nodes_queue.empty())
        {
            std::this_thread::sleep_for(std::chrono::microseconds((int)(env_ptr_->getAlgorithmRunningDelayTime() * 1e6)));
            cur_ = nodes_queue.top();
            nodes_queue.pop();
            cur_->in_close_list = true;
            cur_->in_open_list = false;

            // 避免覆盖起始位置标志
            if (cur_->x != start_x || cur_->y != start_y)
            {
                env_ptr_->setIntGridValByPlanXY(cur_->x, cur_->y, 150, 150, 150);
            }

            if (!visited[cur_->x][cur_->y])
            {
                // 找到未被遍历的网格，退出
                goal_x = cur_->x;
                goal_y = cur_->y;
                find = true;
                break;
            }

            for (auto &side_node : side_points_)
            {
                if (!side_node(env_ptr_, cur_->x, cur_->y, side_x, side_y))
                {
                    continue;
                }

                auto id = side_y * size_x + side_x;
                side = &nodes_[id];

                if (side_x != cur_->x && side_y != cur_->y)
                {
                    side_to_cur_cost = 14;
                }
                else
                {
                    side_to_cur_cost = 10;
                }
                through_cur_cost = side_to_cur_cost + cur_->dist;

                if (side->in_open_list)
                {
                    if (through_cur_cost < side->dist)
                    {
                        side->dist = through_cur_cost;
                        side->parent_node = cur_;
                    }
                    continue;
                }
                else if (side->in_close_list)
                {
                    continue;
                }

                if (side->is_obstacle)
                {
                    continue;
                }

                side_val = env_ptr_->getGridValue(side_x, side_y);
                if (side_val == 0)
                {
                    // 如果此點是障礙物，則跳過
                    side->is_obstacle = true;
                    continue;
                }

                side->id = id;
                side->x = side_x;
                side->y = side_y;
                side->parent_node = cur_;
                side->dist = cur_->dist + side_to_cur_cost;
                side->in_open_list = true;

                nodes_queue.push(side);
            }
        }

        if (find)
        {
            std::vector<environment::PathNode> t_path;
            while (cur_)
            {
                pn.x = cur_->x;
                pn.y = cur_->y;
                t_path.emplace_back(pn);
                cur_ = cur_->parent_node;
            }
            std::reverse(t_path.begin(), t_path.end());
            path.insert(path.end(), t_path.begin(), t_path.end());
        }

        return find;
    }

    environment::Path &BcdWidthDijkstra::getPath()
    {
        return path_;
    }
}
