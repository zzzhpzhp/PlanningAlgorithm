#include "bcd_with_dijkstra.h"
#include "utility.h"

namespace algorithm
{
    void BcdWidthDijkstra::initialize(environment::EnvironmentInterfacePtr &env)
    {
        env_ptr_ = env;

        side_points_.emplace_back(boost::bind(&BcdWidthDijkstra::_get_right, this, _1, _2, _3, _4, _5));
        side_points_.emplace_back(boost::bind(&BcdWidthDijkstra::_get_middle_higher, this, _1, _2, _3, _4, _5));
        side_points_.emplace_back(boost::bind(&BcdWidthDijkstra::_get_left, this, _1, _2, _3, _4, _5));
        side_points_.emplace_back(boost::bind(&BcdWidthDijkstra::_get_middle_lower, this, _1, _2, _3, _4, _5));

//        side_points_.emplace_back(boost::bind(&BcdWidthDijkstra::_get_higher_right, this, _1, _2, _3, _4, _5));
//        side_points_.emplace_back(boost::bind(&BcdWidthDijkstra::_get_higher_left, this, _1, _2, _3, _4, _5));
//        side_points_.emplace_back(boost::bind(&BcdWidthDijkstra::_get_lower_left, this, _1, _2, _3, _4, _5));
//        side_points_.emplace_back(boost::bind(&BcdWidthDijkstra::_get_lower_right, this, _1, _2, _3, _4, _5));

        nodes_ = std::vector<Node>(env_ptr_->getGridXSizeInCells() * env_ptr_->getGridYSizeInCells());

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
        int id_index = 0;
        std::priority_queue<Node*, std::vector<Node*>, Node> nodes_queue;
        Node *cur_ = &nodes_[id_index];
        cur_->x = start_x;
        cur_->y = start_y;
        cur_->dist = 0;
        cur_->id = id_index;
        id_index++;
        nodes_queue.push(cur_);
        environment::PathNode pn{};
        pn.r = 255;
        pn.a = 255;
        bool find = false;
        VisitedTable dj_visited;
        dj_visited[cur_->x][cur_->y] = true;

        while (!nodes_queue.empty())
        {
            std::this_thread::sleep_for(std::chrono::microseconds(200));
            cur_ = nodes_queue.top();
            nodes_queue.pop();

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

            int side_x, side_y;
            uint8_t side_val;
            Node *side;
            for (auto &side_node : side_points_)
            {
                if (!side_node(env_ptr_, cur_->x, cur_->y, side_x, side_y))
                {
                    continue;
                }

                side_val = env_ptr_->getGridValue(side_x, side_y);
                if (dj_visited[side_x][side_y] || side_val == 0)
                {
                    // 如果此點是障礙物，則跳過
                    continue;
                }
                dj_visited[side_x][side_y] = true;

                auto id = id_index++;
                side = &nodes_[id];
                side->id = id;
                side->x = side_x;
                side->y = side_y;
                side->parent_node = cur_;

                if (side->x != cur_->x && side->y != cur_->y)
                {
                    side->dist = cur_->dist + 14;
                }
                else
                {
                    side->dist = cur_->dist + 10;
                }

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
