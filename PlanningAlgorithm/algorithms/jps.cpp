#include "jps.h"

namespace algorithm
{
    void JPS::initialize(environment::EnvironmentInterfacePtr &env, std::string name)
    {
        env_ptr_ = env;
        name_ = name;

        side_points_.emplace_back(boost::bind(&JPS::_get_right, this, _1, _2, _3, _4, _5, 1));
        side_points_.emplace_back(boost::bind(&JPS::_get_middle_higher, this, _1, _2, _3, _4, _5, 1));
        side_points_.emplace_back(boost::bind(&JPS::_get_left, this, _1, _2, _3, _4, _5, 1));
        side_points_.emplace_back(boost::bind(&JPS::_get_middle_lower, this, _1, _2, _3, _4, _5, 1));

        side_points_.emplace_back(boost::bind(&JPS::_get_higher_right, this, _1, _2, _3, _4, _5, 1));
        side_points_.emplace_back(boost::bind(&JPS::_get_higher_left, this, _1, _2, _3, _4, _5, 1));
        side_points_.emplace_back(boost::bind(&JPS::_get_lower_left, this, _1, _2, _3, _4, _5, 1));
        side_points_.emplace_back(boost::bind(&JPS::_get_lower_right, this, _1, _2, _3, _4, _5, 1));

        initialized_ = true;
    }

    void JPS::setGoal(int x, int y)
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

    void JPS::setStart(int x, int y)
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

    bool JPS::planning()
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

        node_cnt_ = 0;
        std::priority_queue<Node*, std::vector<Node*>, NodeCmp> node_stack;
        auto size_x = env_ptr_->getGridXSizeInCells(), size_y = env_ptr_->getGridYSizeInCells();
        int id_index = start_y_ * size_x + start_x_;
        std::vector<Node> nodes(size_x * size_y);
        Node *cur = &nodes[id_index];
        cur->x = start_x_;
        cur->y = start_y_;
        cur->h = calc_h_(cur);
        cur->dist = cur->h;
        node_stack.push(cur);
        cur->in_open_list = true;
        environment::PathNode pn{};
        pn.g = 255;
        pn.a = 255;
        path_.clear();
        int side_x, side_y;
        Node *side;
        float side_to_cur_cost{0.0f};

        auto cost = env_ptr_->getGridValue(start_x_, start_y_);
        if (cost <= environment::INSCRIBED_INFLATED_OBSTACLE)
        {
            return false;
        }

        std::function<bool()> JPS = [&]()->bool
        {
            while (!node_stack.empty() && is_running_.load())
            {
                std::this_thread::sleep_for(std::chrono::microseconds((int)(env_ptr_->getAlgorithmRunningDelayTime() * 1e6)));
                cur = node_stack.top();
                node_stack.pop();
                cur->in_close_list = true;
                cur->in_open_list = false;
                cur->visited = true;

                if (cur->x == goal_x_ && cur->y == goal_y_)
                {
                    std::cout << ">>>>>>>>>>>>>>>>>>> Reach Goal <<<<<<<<<<<<<<<<<<<" <<std::endl;
                    return true;
                }

                if (cur->x != start_x_ || cur->y != start_y_)
                {
                    env_ptr_->setIntGridValByPlanXY(cur->x, cur->y, 100, 100, 100);
                }

                std::this_thread::sleep_for(
                        std::chrono::microseconds((int)(env_ptr_->getAlgorithmRunningDelayTime() * 1e6)));
                Node *t_cur = cur;
                for (const auto& fun : side_points_)
                {
                    cur = t_cur;
                    while (fun(env_ptr_, cur->x, cur->y, side_x, side_y))
                    {
                        std::this_thread::sleep_for(
                                std::chrono::microseconds((int)(env_ptr_->getAlgorithmRunningDelayTime() * 1e6)));
                        auto id = side_y * size_x + side_x;
                        side = &nodes[id];
                        if (side_x != cur->x && side_y != cur->y)
                        {
                            side_to_cur_cost=  14;
                        }
                        else
                        {
                            side_to_cur_cost = 10;
                        }


                        if (side->in_open_list)
                        {
                            auto new_cost = cur->dist + side_to_cur_cost;
                            if (new_cost < side->dist)
                            {
                                side->parent_node = t_cur;
                            }
                            break;
                        }
                        else if (side->in_close_list)
                        {
                            break;
                        }
                        else
                        {
                            if (side->visited)
                            {
                                break;
                            }
                            side->visited = true;
                            side->x = side_x;
                            side->y = side_y;
                            side->dist = cur->dist + side_to_cur_cost;
                        }

                        auto cost = env_ptr_->getGridValue(side->x, side->y);
                        if (cost <= environment::INSCRIBED_INFLATED_OBSTACLE)
                        {
                            break;
                        }

                        if (side->x == goal_x_ && side->y == goal_y_)
                        {
                            std::cout << ">>>>>>>>>>>>>>>>>>> Reach Goal <<<<<<<<<<<<<<<<<<<" <<std::endl;
                            side->parent_node = t_cur;
                            cur = side;
                            return true;
                        }
                        env_ptr_->setIntGridValByPlanXY(side->x, side->y, 100, 100, 100);


                        auto dir = _get_pose(side->x, side->y, cur->x, cur->y);
                        if (_have_forced_neighbor(side->x, side->y, dir) || dir == HR || dir == HL || dir == LL || dir == LR)
                        {
                            side->in_close_list = false;
                            side->in_open_list = true;
                            side->h = calc_h_(side);
                            side->parent_node = t_cur;
                            env_ptr_->setIntGridValByPlanXY(side->x, side->y, 0, 255, 100);

                            node_stack.push(side);
                            node_cnt_++;
                            break;
                        }
                        cur = side;
                    }
                }
            }
            return false;
        };

        auto result = JPS();

        if (result)
        {
            while (cur)
            {
                pn.x = cur->x;
                pn.y = cur->y;
                path_.emplace_back(pn);
                cur = cur->parent_node;
            }
        }
        std::reverse(path_.begin(), path_.end());

        std::cout << "OpenList catched node " << node_cnt_ << std::endl;
        return result;
    }

    environment::Path &JPS::getPath()
    {
        return path_;
    }

    JPS::SIDE_POSE JPS::_get_pose(int cx, int cy, int px, int py)
    {
        if (cx > px && cy < py)
        {
            //右上
            return HR;
        }
        else if (cx < px && cy < py)
        {
            // 左上
            return HL;
        }
        else if (cx < px && cy > py)
        {
            // 左下
            return LL;
        }
        else if (cx > px && cy > py)
        {
            // 右下
            return LR;
        }
        else if (cx < px)
        {
            // 左
            return L;
        }
        else if(cx > px)
        {
            // 右
            return R;
        }
        else if (cy < py)
        {
            // 上
            return MH;
        }
        else if (cy > py)
        {
            // 下
            return ML;
        }
        else
        {
            return R;
        }
    }

    bool JPS::_have_forced_neighbor(int cx, int cy, JPS::SIDE_POSE pos)
    {
        int mh{-1}, ml{-1}, l{-1}, r{-1};
        int tx, ty;
        if (_get_middle_higher(env_ptr_, cx, cy, tx, ty))
        {
            mh = env_ptr_->getGridValue(tx, ty);
        }
        if (_get_middle_lower(env_ptr_, cx, cy, tx, ty))
        {
            ml = env_ptr_->getGridValue(tx, ty);
        }
        if (_get_left(env_ptr_, cx, cy, tx, ty))
        {
            l = env_ptr_->getGridValue(tx, ty);
        }
        if (_get_right(env_ptr_, cx, cy, tx, ty))
        {
            r = env_ptr_->getGridValue(tx, ty);
        }

        if (pos == HR)
        {
            // 右上
            return _is_obstacle(l) || _is_obstacle(ml);
        }
        else if (pos == HL)
        {
            // 左上
            return _is_obstacle(r) || _is_obstacle(ml);
        }
        else if (pos == LL)
        {
            // 左下
            return _is_obstacle(r) || _is_obstacle(mh);
        }
        else if (pos == LR)
        {
            // 右下
            return _is_obstacle(l) || _is_obstacle(mh);
        }
        else if (pos == L)
        {
            return (_is_obstacle(mh) || _is_obstacle(ml)) && !_is_obstacle(l);
        }
        else if ( pos == R)
        {
            return (_is_obstacle(mh) || _is_obstacle(ml)) && !_is_obstacle(r);
        }
        else if (pos == MH)
        {

            return (_is_obstacle(l) || _is_obstacle(r)) && !_is_obstacle(mh);
        }
        else if (pos == ML)
        {
            return (_is_obstacle(l) || _is_obstacle(r)) && !_is_obstacle(ml);
        }
        else
        {
            std::cerr << "Should not be here." << std::endl;
        }
    }
}