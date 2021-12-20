#include "dijkstra.h"

namespace algorithm
{
    void Dijkstra::initialize(environment::EnvironmentInterfacePtr &env, std::string name)
    {
        env_ptr_ = env;
        name_ = name;

        side_points_.emplace_back(boost::bind(&Dijkstra::_get_right, this, _1, _2, _3, _4, _5, 1));
        side_points_.emplace_back(boost::bind(&Dijkstra::_get_middle_higher, this, _1, _2, _3, _4, _5, 1));
        side_points_.emplace_back(boost::bind(&Dijkstra::_get_left, this, _1, _2, _3, _4, _5, 1));
        side_points_.emplace_back(boost::bind(&Dijkstra::_get_middle_lower, this, _1, _2, _3, _4, _5, 1));

        side_points_.emplace_back(boost::bind(&Dijkstra::_get_higher_right, this, _1, _2, _3, _4, _5, 1));
        side_points_.emplace_back(boost::bind(&Dijkstra::_get_higher_left, this, _1, _2, _3, _4, _5, 1));
        side_points_.emplace_back(boost::bind(&Dijkstra::_get_lower_left, this, _1, _2, _3, _4, _5, 1));
        side_points_.emplace_back(boost::bind(&Dijkstra::_get_lower_right, this, _1, _2, _3, _4, _5, 1));

        initialized_ = true;
    }

    void Dijkstra::setGoal(int x, int y)
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

    void Dijkstra::setStart(int x, int y)
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

    bool Dijkstra::planning()
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


        std::priority_queue<Node*, std::vector<Node*>, Node> node_stack;
        auto size_x = env_ptr_->getGridXSizeInCells(), size_y = env_ptr_->getGridYSizeInCells();
        int id_index = start_y_ * size_x + start_x_;
        std::vector<Node> nodes(size_x * size_y);
        Node *cur_ = &nodes[id_index];
        cur_->x = start_x_;
        cur_->y = start_y_;
        cur_->dist = 0;
        cur_->id = id_index;
        node_stack.push(cur_);
        cur_->in_open_list = true;
        environment::PathNode pn{};
        pn.g = 255;
        pn.a = 255;
        path_.clear();
        int side_x, side_y;
        uint8_t side_val;
        Node *side;
        int side_to_cur_cost{};
        int through_cur_cost{};

        std::function<bool()> Dijkstra = [&]()->bool
        {
            while (!node_stack.empty() && is_running_.load())
            {
                std::this_thread::sleep_for(std::chrono::microseconds((int)(env_ptr_->getAlgorithmRunningDelayTime() * 1e6)));
                cur_ = node_stack.top();
                node_stack.pop();
                cur_->in_close_list = true;
                cur_->in_open_list = false;

                if (cur_->x == goal_x_ && cur_->y == goal_y_)
                {
                    return true;
                }

                if (cur_->x != start_x_ || cur_->y != start_y_)
                {
                    env_ptr_->setIntGridValByPlanXY(cur_->x, cur_->y, 100, 100, 100);
                }

                for (auto &side_node : side_points_)
                {
                    if (!side_node(env_ptr_, cur_->x, cur_->y, side_x, side_y))
                    {
                        continue;
                    }

                    auto id = side_y * size_x + side_x;
                    side = &nodes[id];

                    if (side_x != cur_->x && side_y != cur_->y)
                    {
                        side_to_cur_cost=  14;
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
                            side->parent_node = cur_;
                            side->dist = through_cur_cost;
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

                    // 此点是新节点
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

                    node_stack.push(side);
                }
            }

            return false;
        };

        auto result = Dijkstra();

        if (result)
        {
            while (cur_)
            {
                pn.x = cur_->x;
                pn.y = cur_->y;
                path_.emplace_back(pn);
                cur_ = cur_->parent_node;
            }
        }

        return result;
    }

    environment::Path &Dijkstra::getPath()
    {
        return path_;
    }
}
