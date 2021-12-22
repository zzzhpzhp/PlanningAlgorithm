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

        dijkstra_side_points_.emplace_back(boost::bind(&BcdWithFootprint::_get_right, this, _1, _2, _3, _4, _5, 1));
        dijkstra_side_points_.emplace_back(boost::bind(&BcdWithFootprint::_get_middle_higher, this, _1, _2, _3, _4, _5, 1));
        dijkstra_side_points_.emplace_back(boost::bind(&BcdWithFootprint::_get_left, this, _1, _2, _3, _4, _5, 1));
        dijkstra_side_points_.emplace_back(boost::bind(&BcdWithFootprint::_get_middle_lower, this, _1, _2, _3, _4, _5, 1));

        dijkstra_side_points_.emplace_back(boost::bind(&BcdWithFootprint::_get_higher_right, this, _1, _2, _3, _4, _5, 1));
        dijkstra_side_points_.emplace_back(boost::bind(&BcdWithFootprint::_get_higher_left, this, _1, _2, _3, _4, _5, 1));
        dijkstra_side_points_.emplace_back(boost::bind(&BcdWithFootprint::_get_lower_left, this, _1, _2, _3, _4, _5, 1));
        dijkstra_side_points_.emplace_back(boost::bind(&BcdWithFootprint::_get_lower_right, this, _1, _2, _3, _4, _5, 1));

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

        auto val = env_ptr_->getGridValue(start_x_, start_y_);
        if (val == 0)
        {
            return false;
        }
        if (_position_validation(start_x_, start_y_, limiting_index_l, limiting_index_h))
        {
            _mark_cleaned(start_x_, start_y_, limiting_index_l, limiting_index_h);
            env_ptr_->setIntGridValByPlanXY(start_x_, start_y_, 100, 100, 100);
            in_path_[start_x_][start_y_] = true;
        }
        pn.x = start_x_;
        pn.y = start_y_;
        path_.emplace_back(pn);

        std::function<bool(int, int)> BcdWithFootprint = [&](int x, int y)->bool
        {
            if (!is_running_)
            {
                return false;
            }
            visited_[x][y] = true;
            in_path_[x][y] = true;
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
                if (side_y != y)
                {
                    // 从高y开始检查
                    auto h = std::max(side_y, y);
                    if (!_position_validation(side_x, h, limiting_index_l, limiting_index_h))
                    {
                        continue;
                    }
                }

                valid = true;
                _mark_cleaned(side_x, side_y, limiting_index_l, limiting_index_h);
                env_ptr_->setIntGridValByPlanXY(side_x, side_y, 100, 100, 100);
                pn.x = side_x;
                pn.y = side_y;
                path_.emplace_back(pn);
                if (!BcdWithFootprint(side_x, side_y))
                {
                    return false;
                }
            }
            if (!valid)
            {
                // Dead point
                env_ptr_->setIntGridValByPlanXY(x, y, 100, 0, 0);
                if (!_dijkstra(x, y, x, y, visited_, path_))
                {
                    FL_PRINT
                    return false;
                }
                if (!BcdWithFootprint(x, y))
                {
                    return false;
                }
            }
            return true;
        };

         BcdWithFootprint(start_x_, start_y_);
         return true;
    }

    bool BcdWithFootprint::_dijkstra(int start_x, int start_y, int &goal_x, int &goal_y,
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
        int side_to_cur_cost{0};
        int through_cur_cost{0};
        int side_x, side_y;
        uint8_t side_val;
        Node *side;

        while (!nodes_queue.empty() && is_running_.load())
        {
            std::this_thread::sleep_for(std::chrono::microseconds((int)(env_ptr_->getAlgorithmRunningDelayTime() * 1e6)));
            cur_ = nodes_queue.top();
            nodes_queue.pop();
            cur_->in_close_list = true;
            cur_->in_open_list = false;

            // 避免覆盖起始位置标志
            if (cur_->x != start_x || cur_->y != start_y)
            {
//                env_ptr_->setIntGridValByPlanXY(cur_->x, cur_->y, 150, 150, 150);
            }

            int limit_x, limit_y;
            if (!visited[cur_->x][cur_->y] && _position_validation(cur_->x, cur_->y, limit_x, limit_y) &&
                abs(cur_->y - start_y) % robot_radius_ == 0)
            {
                // 找到未被遍历的网格，退出
                goal_x = cur_->x;
                goal_y = cur_->y;
                find = true;
                break;
            }

            for (auto &side_node : dijkstra_side_points_)
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
            if (!t_path.empty())
            {
                // 去掉起点
                t_path.pop_back();
                std::reverse(t_path.begin(), t_path.end());
                path.insert(path.end(), t_path.begin(), t_path.end());
            }
        }

        return find;
    }

    environment::Path& BcdWithFootprint::getPath()
    {
        return path_;
    }

    bool BcdWithFootprint::_position_validation(int x, int y, int &limiting_index_l, int &limiting_index_h)
    {
        limiting_index_l = limiting_index_h = 0;
        bool res{false};
        for (int i = -0; i >= -robot_radius_; i--)
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
                res = false;
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
//            cleaned_[tx][ty] = true;
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