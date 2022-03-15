#include "expander.h"
#define DISPLAY_PROCESS 0
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

        max_x_ = max_y_ = std::numeric_limits<int>::min();
        min_x_ = min_y_ = std::numeric_limits<int>::max();

        auto size_x = env_ptr_->getGridXSizeInCells(), size_y = env_ptr_->getGridYSizeInCells();

        std::priority_queue<Node*, std::vector<Node*>, NodeCmp> node_stack;
        std::vector<Node> nodes(size_x * size_y);
        auto start_cost = env_ptr_->getGridValue(start_x_, start_y_);
        Node *cur = &nodes[start_y_ * size_x + start_x_];
        // 初始化第一个点
        cur->x = start_x_;
        cur->y = start_y_;
        cur->cost = start_cost;
        cur->dist = 0;
        cur->id = start_y_ * size_x + start_x_;
        node_stack.push(cur);
        std::unordered_map<int, std::unordered_map<int, bool>> visited{0};
        environment::PathNode pn{};
        pn.g = 255;
        pn.a = 255;
        path_.clear();

        visited[cur->x][cur->y] = true;

        std::function<bool()> bfs = [&]()->bool
        {
            while (!node_stack.empty() && env_ptr_->isRunning())
            {
                std::this_thread::sleep_for(std::chrono::microseconds((int)(env_ptr_->getAlgorithmRunningDelayTime() * 1e6)));
                cur = node_stack.top();
                node_stack.pop();

                max_x_ = std::max(max_x_, cur->x);
                min_x_ = std::min(min_x_, cur->x);
                max_y_ = std::max(max_y_, cur->y);
                min_y_ = std::min(min_y_, cur->y);

                if (cur->x != start_x_ || cur->y != start_y_)
                {
#if DISPLAY_PROCESS
                    env_ptr_->setIntGridValueByGridXY(cur->x, cur->y, 100, 100, 100);
#endif
                }

                if (step_process_ != nullptr)
                {
                    step_process_(cur->x, cur->y, cur->cost);
                }

                if (should_terminate_ != nullptr)
                {
                    if (should_terminate_(cur->x, cur->y, cur->cost))
                    {
                        return true;
                    }
                }

                int side_x, side_y;
                uint8_t side_val;
                Node *side;
                for (auto &side_node : side_points_)
                {
                    if (!side_node(env_ptr_, cur->x, cur->y, side_x, side_y))
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

                    auto id = side_y * size_x + side_x;
                    side = &nodes[id];
                    visited[side_x][side_y] = true;

                    float side_to_cur_cost;
                    if (side_x != cur->x && side_y != cur->y)
                    {
                        side_to_cur_cost = 14;
                    }
                    else
                    {
                        side_to_cur_cost = 10;
                    }

                    float through_cur_cost = side_to_cur_cost + cur->dist + (255 - side_val) * map_cost_scale_;
                    if (side->in_open_list)
                    {
                        if (through_cur_cost < side->dist)
                        {
                            side->parent_node = cur;
                            side->dist = through_cur_cost;
                        }
                        continue;
                    }
                    else if (side->in_close_list)
                    {
                        continue;
                    }

                    side->id = id;
                    side->x = side_x;
                    side->y = side_y;
                    side->cost = side_val;
                    side->parent_node = cur;
                    side->dist = cur->dist + side_to_cur_cost + (255 - side_val) * map_cost_scale_;
                    node_stack.push(side);
                }
            }
            return false;
        };

        auto result = bfs();

        if (result)
        {
            while (cur)
            {
                pn.x = cur->x;
                pn.y = cur->y;
                path_.emplace_back(pn);
                cur = cur->parent_node;
            }
            std::reverse(path_.begin(), path_.end());
        }

        return result;
    }
}
