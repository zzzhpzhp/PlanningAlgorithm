#include "bcd_with_footprint.h"
#define USE_DIJKSTRA 1
#define DIJKSTRA_SEARCH_DISPLAY 0
#define LEAP_SEARCH_ENABLE 1
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

        reach_judge_ = boost::bind(&BcdWithFootprint::_goal_reached, this, _1, _2);
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
        search_leap_ = false;

        auto val = env_ptr_->getGridValue(start_x_, start_y_);
        if (val == 0)
        {
            return false;
        }
        _position_validation(start_x_, start_y_);
        _mark_cleaned(start_x_, start_y_);

        std::deque<std::tuple<int, int>> node_queue;
        node_queue.emplace_back(start_x_, start_y_);
        visited_[start_x_][start_y_] = true;
        cleaned_[start_x_][start_y_] = true;

        int side_x, side_y;
        uint8_t side_val;
        bool valid = true;
        int x{start_x_}, y{start_y_};
        while(!node_queue.empty() && is_running_.load())
        {
            std::this_thread::sleep_for(
                    std::chrono::microseconds((int)(env_ptr_->getAlgorithmRunningDelayTime() * 1e6)));
            if (!valid)
            {
                env_ptr_->setIntGridValByPlanXY(x, y, 100, 0, 0);
#if USE_DIJKSTRA
                if (!_dijkstra(x, y, x, y, visited_, path_))
                {
#if LEAP_SEARCH_ENABLE
                    search_leap_ = true;
                    if (!_dijkstra(x, y, x, y, visited_, path_))
                    {
                        break;
                    }
#else
                    break;
#endif
                }
                env_ptr_->setIntGridValByPlanXY(x, y, 100, 100, 100);
                visited_[x][y] = true;
                cleaned_[x][y] = true;
                pn.x = x;
                pn.y = y;
                path_.emplace_back(pn);
#else
                auto top = node_queue.back();
                node_queue.pop_back();
                x = std::get<0>(top);
                y = std::get<1>(top);
#endif
                _position_validation(x, y);
                _mark_cleaned(x, y);
            }
            valid = false;
//            side_points_.clear();
//            if (y > start_y_)
//            {
//                side_points_.emplace_back(boost::bind(&BcdWithFootprint::_get_left, this, _1, _2, _3, _4, _5, 1));
//                side_points_.emplace_back(boost::bind(&BcdWithFootprint::_get_right, this, _1, _2, _3, _4, _5, 1));
//            }
//            else
//            {
//                side_points_.emplace_back(boost::bind(&BcdWithFootprint::_get_right, this, _1, _2, _3, _4, _5, 1));
//                side_points_.emplace_back(boost::bind(&BcdWithFootprint::_get_left, this, _1, _2, _3, _4, _5, 1));
//            }
//            if (x > start_x_)
//            {
//                side_points_.emplace_back(boost::bind(&BcdWithFootprint::_get_middle_lower, this, _1, _2, _3, _4, _5, robot_radius_));
//                side_points_.emplace_back(boost::bind(&BcdWithFootprint::_get_middle_higher, this, _1, _2, _3, _4, _5, robot_radius_));
//            }
//            else
//            {
//                side_points_.emplace_back(boost::bind(&BcdWithFootprint::_get_middle_higher, this, _1, _2, _3, _4, _5, robot_radius_));
//                side_points_.emplace_back(boost::bind(&BcdWithFootprint::_get_middle_lower, this, _1, _2, _3, _4, _5, robot_radius_));
//            }
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
                if (search_leap_ && cleaned_[side_x][side_y])
                {
                    continue;
                }
                if (side_y != y)
                {
                    // 从高y开始检查
                    auto h = std::max(side_y, y);
                    if (!_position_validation(side_x, h))
                    {
                        continue;
                    }
                }
                else
                {
                    _position_validation(side_x, side_y);
                }

                valid = true;
                _mark_cleaned(side_x, side_y);
                env_ptr_->setIntGridValByPlanXY(side_x, side_y, 100, 100, 100);
                node_queue.emplace_back(side_x, side_y);
                visited_[side_x][side_y] = true;
                cleaned_[side_x][side_y] = true;

                x = side_x;
                y = side_y;

                pn.x = x;
                pn.y = y;
                path_.emplace_back(pn);
                break;
            }
        }
        return true;
    }

    bool BcdWithFootprint::_dijkstra(int start_x, int start_y, int &goal_x, int &goal_y,
                                     VisitedTable& visited, std::vector<environment::PathNode> &path)
    {
        auto size_x = env_ptr_->getGridXSizeInCells(), size_y = env_ptr_->getGridYSizeInCells();
        nodes_ = std::vector<Node>(size_x * size_y);
        std::priority_queue<Node*, std::vector<Node*>, Node> nodes_queue;
        int id_index = start_y * size_x + start_x;
        Node *cur = &nodes_[id_index];
        cur->x = start_x;
        cur->y = start_y;
        cur->dist = 0;
        nodes_queue.push(cur);
        environment::PathNode pn{};
        if (!search_leap_)
        {
            pn.r = 255;
        }
        else
        {
            pn.r = 0;
            pn.b = 255;
        }
        pn.a = 50;
        bool find = false;
        int side_to_cur_cost{0};
        int through_cur_cost{0};
        int side_x, side_y;
        uint8_t side_val;
        Node *side;

        while (!nodes_queue.empty() && is_running_.load())
        {
            std::this_thread::sleep_for(std::chrono::microseconds((int)(env_ptr_->getAlgorithmRunningDelayTime() * 1e6)));
            cur = nodes_queue.top();
            nodes_queue.pop();
            cur->in_close_list = true;
            cur->in_open_list = false;

#if DIJKSTRA_SEARCH_DISPLAY
            // 避免覆盖起始位置标志
            if (cur->x != start_x || cur->y != start_y)
            {
                env_ptr_->setIntGridValByPlanXY(cur->x, cur->y, 100, 100, 100, 50);
            }
#endif
            if (reach_judge_(cur->x, cur->y))
            {
                // 找到未被遍历的网格，退出
                goal_x = cur->x;
                goal_y = cur->y;
                find = true;
                break;
            }

            for (auto &side_node : dijkstra_side_points_)
            {
                if (!side_node(env_ptr_, cur->x, cur->y, side_x, side_y))
                {
                    continue;
                }

                auto id = side_y * size_x + side_x;
                side = &nodes_[id];

                if (side_x != cur->x && side_y != cur->y)
                {
                    side_to_cur_cost = 14;
                }
                else
                {
                    side_to_cur_cost = 10;
                }
                through_cur_cost = side_to_cur_cost + cur->dist;

                if (side->in_open_list)
                {
                    if (through_cur_cost < side->dist)
                    {
                        side->dist = through_cur_cost;
                        side->parent_node = cur;
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

                side->x = side_x;
                side->y = side_y;
                side->parent_node = cur;
                side->dist = cur->dist + side_to_cur_cost;
                side->in_open_list = true;

                nodes_queue.push(side);
            }
        }

        if (find)
        {
            std::vector<environment::PathNode> t_path;
            while (cur)
            {
                pn.x = cur->x;
                pn.y = cur->y;
                t_path.emplace_back(pn);
                cur = cur->parent_node;
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

    bool BcdWithFootprint::_position_validation(int x, int y)
    {
//        limiting_index_l = limiting_index_h = 0;
        bool res{false};
        for (int i = -1; i > -robot_radius_; i--)
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
//            limiting_index_l = i;
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
//            limiting_index_h = i;
        }
        return res;
    }

    void BcdWithFootprint::_mark_cleaned(int x, int y)
    {
        for (int i = -1; i > -robot_radius_ ; i--)
        {
            auto tx = x;
            auto ty = i + y;
            if (!env_ptr_->insideGrid(tx, ty))
            {
                continue;
            }
            if (!visited_[tx][ty])
            {
                auto val = env_ptr_->getGridValue(tx, ty);
                if (val == 0)
                {
                    break;
                }
                env_ptr_->setIntGridValByPlanXY(tx, ty, 150, 150, 150);
            }
            cleaned_[tx][ty] = true;
        }

        for (int i = 1; i < robot_radius_; i++)
        {
            auto tx = x;
            auto ty = i + y;
            if (!env_ptr_->insideGrid(tx, ty))
            {
                continue;
            }
            if (!visited_[tx][ty])
            {
                auto val = env_ptr_->getGridValue(tx, ty);
                if (val == 0)
                {
                    break;
                }
                env_ptr_->setIntGridValByPlanXY(tx, ty, 150, 150, 150);
            }
            cleaned_[tx][ty] = true;
        }
    }

    bool BcdWithFootprint::_goal_reached(int x, int y)
    {
        auto val = env_ptr_->getGridValue(x, y);
        if (val == 0)
        {
            return false;
        }

        if (!search_leap_)
        {
            return (!visited_[x][y] && !cleaned_[x][y] && abs(y - start_y_) % robot_radius_ == 0);
        }
        else
        {
            return !visited_[x][y] && !cleaned_[x][y];
        }
    }
}