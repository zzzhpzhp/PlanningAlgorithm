#include "region_manager.h"

namespace algorithm
{

    bool RegionManager::planning()
    {
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

        struct Node
        {
            int x, y;
            int id;
            int dist;
            Node *parent_node;

            bool operator()(const Node* a, const Node* n) const
            {
                return a->dist > n->dist;
            };
        };

        struct NodeCmp
        {
            bool operator()(const Node* a, const Node* n) const
            {
                return a->dist > n->dist;
            };
        };

        std::priority_queue<Node*, std::vector<Node*>, NodeCmp> node_stack;
        int id_index = 0;
        std::vector<Node> nodes(env_ptr_->getGridXSizeInCells() * env_ptr_->getGridYSizeInCells());
        Node *cur_ = &nodes[id_index];
        cur_->x = start_x_;
        cur_->y = start_y_;
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
            while (!node_stack.empty() && is_running_.load())
            {
                std::this_thread::sleep_for(std::chrono::microseconds((int)(env_ptr_->getAlgorithmRunningDelayTime() * 1e6)));
                cur_ = node_stack.top();
                node_stack.pop();

                if (cur_->x != start_x_ || cur_->y != start_y_)
                {
                    env_ptr_->setIntGridValByPlanXY(cur_->x, cur_->y, 100, 100, 100);
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
                    if (visited[side_x][side_y] || side_val <= environment::INSCRIBED_INFLATED_OBSTACLE)
                    {
                        // 如果此點已被訪問過或是障礙物，則跳過
                        continue;
                    }

                    visited[side_x][side_y] = true;

                    auto id = id_index++;
                    side = &nodes[id];
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

    environment::Path &RegionManager::getPath()
    {
        return path_;
    }

    bool RegionManager::_is_boundary(int x, int y)
    {
        if ((x - region_origin_.x) % region_size_ == 0 && (y - region_origin_.y) % region_size_ == 0)
        {
            return true;
        }
        return false;
    }

    std::string RegionManager::_get_region_id(int x, int y)
    {
        for (const auto &reg : regions_)
        {
            const auto& r = reg.second;
            if (x < r.xh && y < r.yh && x >= r.xl && y >= r.yl)
            {
                return r.id;
            }
        }

        return std::string{};
    }

    bool RegionManager::addRegion(int x, int y)
    {
        Region r;
        int x_normalize = x / region_size_;
        int y_normalize = y / region_size_;

        r.xh = x_normalize * region_size_ + region_size_;
        r.xl = x_normalize * region_size_;
        r.yh = y_normalize * region_size_ + region_size_;
        r.yl = y_normalize * region_size_;

        auto id = _gen_id_for_region(r);
        if (regions_.find(id) == regions_.end())
        {
            regions_[id] = r;
            return true;
        }
        return false;
    }
}