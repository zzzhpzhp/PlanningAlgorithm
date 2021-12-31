#include "expander.h"

namespace algorithm
{

    Expander::Expander()
    {
        side_points_.emplace_back(boost::bind(&Expander::_get_right, this, _1, _2, _3, _4, _5, 1));
        side_points_.emplace_back(boost::bind(&Expander::_get_higher_right, this, _1, _2, _3, _4, _5, 1));
        side_points_.emplace_back(boost::bind(&Expander::_get_middle_higher, this, _1, _2, _3, _4, _5, 1));
        side_points_.emplace_back(boost::bind(&Expander::_get_higher_left, this, _1, _2, _3, _4, _5, 1));
        side_points_.emplace_back(boost::bind(&Expander::_get_left, this, _1, _2, _3, _4, _5, 1));
        side_points_.emplace_back(boost::bind(&Expander::_get_lower_left, this, _1, _2, _3, _4, _5, 1));
        side_points_.emplace_back(boost::bind(&Expander::_get_middle_lower, this, _1, _2, _3, _4, _5, 1));
        side_points_.emplace_back(boost::bind(&Expander::_get_lower_right, this, _1, _2, _3, _4, _5, 1));
    }

    void Expander::initialize(environment::EnvironmentInterfacePtr &env, std::string name)
    {
        env_ptr_ = env;
        name_ = name;

        initialized_ = true;
    }

    void Expander::setStart(int x, int y)
    {
        start_x_ = x;
        start_y_ = y;
    }

    void Expander::setShouldTerminate(std::function<bool(int x, int y, unsigned char cost)> fun)
    {
        should_terminate_ = fun;
    }

    void Expander::setStepProcess(std::function<bool(int, int, unsigned char)> fun)
    {
        step_process_ = fun;
    }

    void Expander::setPoseValidation(std::function<bool(int, int, unsigned char)> fun)
    {
        pose_validation_ = fun;
    }

    environment::Path &Expander::getPath()
    {
        return path_;
    }

    bool Expander::expand()
    {
        std::cout << "expander " << std::endl;
        if (!initialized_)
        {
            std::cerr << "Should initialize first." << std::endl;
            return false;
        }

        if (!env_ptr_->insideGrid(start_x_, start_y_))
        {
            std::cerr << "Start or Goal pose out of bound." << std::endl;
            return false;
        }

        std::cout << "Start pose: [" << start_x_ << ", " << start_y_ << "]" << std::endl;

        std::priority_queue<Node*, std::vector<Node*>, NodeCmp> node_stack;
        int id_index = 0;
        std::vector<Node> nodes(env_ptr_->getGridXSizeInCells() * env_ptr_->getGridYSizeInCells());
        auto start_cost = env_ptr_->getGridValue(start_x_, start_y_);
        Node *cur_ = &nodes[id_index];
        cur_->x = start_x_;
        cur_->y = start_y_;
        cur_->cost = start_cost;
        cur_->dist = 0;
        cur_->id = id_index;
        id_index++;
        node_stack.push(cur_);
        std::unordered_map<int, std::unordered_map<int, bool>> visited;
        environment::PathNode pn{};
        pn.g = 255;
        pn.a = 255;
        path_.clear();

        visited[cur_->x][cur_->y] = true;

        std::function<bool()> bfs = [&]()->bool
        {
            while (!node_stack.empty() && env_ptr_->isRunning())
            {
                std::this_thread::sleep_for(std::chrono::microseconds((int)(env_ptr_->getAlgorithmRunningDelayTime() * 1e6)));
                cur_ = node_stack.top();
                node_stack.pop();

                if (cur_->x != start_x_ || cur_->y != start_y_)
                {
                    env_ptr_->setIntGridValByPlanXY(cur_->x, cur_->y, 100, 100, 100);
                }

                if (step_process_ != nullptr)
                {
                    step_process_(cur_->x, cur_->y, cur_->cost);
                }

                if (should_terminate_ != nullptr)
                {
                    if (should_terminate_(cur_->x, cur_->y, cur_->cost))
                    {
                        return true;
                    }
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
                    if (visited[side_x][side_y] || side_val <= environment::INSCRIBED_INFLATED_OBSTACLE ||
                        (pose_validation_ && !pose_validation_(side_x, side_y, side_val)))
                    {
                        // 如果此點已被訪問過或是障礙物，則跳過
                        continue;
                    }

                    visited[side_x][side_y] = true;

                    float side_to_cur_cost;
                    if (side_x != cur_->x && side_y != cur_->y)
                    {
                        side_to_cur_cost = 14;
                    }
                    else
                    {
                        side_to_cur_cost = 10;
                    }

                    float through_cur_cost = side_to_cur_cost + cur_->dist + 255 - side_val;
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

                    auto id = id_index++;
                    side = &nodes[id];
                    side->id = id;
                    side->x = side_x;
                    side->y = side_y;
                    side->cost = side_val;
                    side->parent_node = cur_;
                    side->dist =  cur_->dist + side_to_cur_cost + 255 - side_val;
                    node_stack.push(side);
                }
            }

            return false;
        };

        auto result = bfs();

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
}