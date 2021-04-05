#include "bfs.h"

namespace algorithm
{
    void Bfs::initialize(environment::EnvironmentInterfacePtr &env)
    {
        env_ptr_ = env;

        side_points_.emplace_back(boost::bind(&Bfs::_get_right, this, _1, _2, _3, _4, _5));
        side_points_.emplace_back(boost::bind(&Bfs::_get_higher_right, this, _1, _2, _3, _4, _5));
        side_points_.emplace_back(boost::bind(&Bfs::_get_middle_higher, this, _1, _2, _3, _4, _5));
        side_points_.emplace_back(boost::bind(&Bfs::_get_higher_left, this, _1, _2, _3, _4, _5));
        side_points_.emplace_back(boost::bind(&Bfs::_get_left, this, _1, _2, _3, _4, _5));
        side_points_.emplace_back(boost::bind(&Bfs::_get_lower_left, this, _1, _2, _3, _4, _5));
        side_points_.emplace_back(boost::bind(&Bfs::_get_middle_lower, this, _1, _2, _3, _4, _5));
        side_points_.emplace_back(boost::bind(&Bfs::_get_lower_right, this, _1, _2, _3, _4, _5));

        initialized_ = true;
    }

    void Bfs::setGoal(int x, int y)
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

    void Bfs::setStart(int x, int y)
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

    bool Bfs::planning()
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

        struct Node
        {
            int x, y;
            int id, parent_id;
            int dist;
            bool operator<(const Node& n) const
            {
                return dist > n.dist;
            }
        };
        std::priority_queue<Node> node_stack;
        int id_index = 0;
        Node cur_;
        cur_.x = start_x_;
        cur_.y = start_y_;
        cur_.dist = 0;
        cur_.id = id_index++;
        cur_.parent_id = -1;
        node_stack.push(cur_);
        std::unordered_map<int, std::unordered_map<int, bool>> visited;
        visited[cur_.x][cur_.y] = true;
        environment::PathNode pn{};
        pn.g = 255;
        pn.a = 255;
        path_.clear();

        std::function<bool()> bfs = [&]()->bool
        {
            while (!node_stack.empty())
            {
                std::this_thread::sleep_for(std::chrono::microseconds(200));
                cur_ = node_stack.top();
                node_stack.pop();

                pn.x = cur_.x;
                pn.y = cur_.y;
                path_.emplace_back(pn);

                if (cur_.x == goal_x_ && cur_.y == goal_y_)
                {
                    return true;
                }

                if (cur_.x == goal_x_ && cur_.y == goal_y_)
                {
                    return true;
                }

                if (cur_.x != start_x_ || cur_.y != start_y_)
                {
                    env_ptr_->setIntGridValByPlanXY(cur_.x, cur_.y, 100, 100, 100);
                }

                int side_x, side_y;
                uint8_t side_val;
                Node side;
                for (auto &side_node : side_points_)
                {
                    if (!side_node(env_ptr_, cur_.x, cur_.y, side_x, side_y))
                    {
                        continue;
                    }
                    side_val = env_ptr_->getGridValue(side_x, side_y);
                    if (visited[side_x][side_y] || side_val == 0)
                    {
                        continue;
                    }
                    visited[side_x][side_y] = true;

                    side.x = side_x;
                    side.y = side_y;
                    if (side.x != cur_.x && side.y != cur_.y)
                    {
                        side.dist = cur_.dist + 14;
                    }
                    else
                    {
                        side.dist = cur_.dist + 10;
                    }
                    side.parent_id = cur_.id;
                    side.id = id_index++;

                    node_stack.push(side);
                }
            }

            return false;
        };

        auto result = bfs();
        return result;
    }

    environment::Path &Bfs::getPath()
    {
        return path_;
    }
}
