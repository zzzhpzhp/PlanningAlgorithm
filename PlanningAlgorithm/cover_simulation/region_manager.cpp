#include "region_manager.h"

namespace algorithm
{
    void RegionManager::initialize(environment::EnvironmentInterfacePtr &env, std::string name)
    {
        env_ptr_ = env;
        name_ = name;
        expander_.initialize(env, name + "Expander");
        helper_expander_.initialize(env, name + "HelperExpander");
        cover_.initialize(env, name + "Cover");
        cover_.setPoseValidation(boost::bind(&RegionManager::isInsideCurrentRegion, this, _1, _2));

        initialized_ = true;
    }

    bool RegionManager::planning()
    {
        // 复位全局已覆盖标记
        cover_.resetGlobalCleaned();
        cover_.setPoseValidation(boost::bind(&RegionManager::isInsideCurrentRegion, this, _1, _2));

        // 边界判定函数
        std::function<bool(int, int, unsigned char)> reach_bound =
        [&](int x, int y, unsigned char cost)->bool
        {
            return _is_boundary(x, y, *current_region_);
        };

        // 新区域到达判定函数
        std::function<bool(int, int, unsigned char)> reach_new_region =
        [&](int x, int y, unsigned char cost)->bool
        {
            // 点是否在指定区域内
            if (!_is_inside(x, y, current_region_))
            {
                // 点不在当前区域内，说明搜索到了当前区域外的其他区域

                auto new_region = getRegionById(x, y);
                // 判断点是否在覆盖过的区域
                if (!cleaned_region_[_gen_region_id(new_region)])
                {
                    // 点所在的区域没有被覆盖过
                    return true;
                }
                else
                {
                    // 点所在的区域被覆盖过, 则检查此区域是否还有未覆盖区域

                    if (cover_.isGlobalCleaned(x, y))
                    {
                        // 当前点已经被覆盖过
                        return false;
                    }

                    // 当前点未被覆盖,以此点为起点，广度优先搜索此区域，检查未被覆盖的点的数量，看看未被覆盖的点的数量是否达到标准

                    int need_cover{0};
                    // 可清扫点记数
                    std::function<bool(int, int, unsigned char)> need_cover_count = [&](int tx, int ty, unsigned char cost)->bool
                    {
                        need_cover++;
                        return true;
                    };
                    // 新区域点判断
                    std::function<bool(int, int, unsigned char)> inside_new_region = [&](int tx, int ty, unsigned char cost)->bool
                    {
                        // 判断点是否在新区域内
                        if (!_is_inside(tx, ty, &new_region))
                        {
                            return false;
                        }

                        // 判断新区域内的点是否已经被覆盖
                        if (cover_.isGlobalCleaned(tx, ty))
                        {
                            return false;
                        }

                        // 是新区域内的未被覆盖的点
                        return true;
                    };
                    // 复位各个条件函数为nullptr
                    helper_expander_.reset();
                    helper_expander_.setStart(x, y);
                    // 设置每对每个加入过open_list中的点的操作为计数
                    helper_expander_.setStepProcess(need_cover_count);
                    // 设置可行性附加判断条件为新区域中的未清扫点
                    helper_expander_.setPoseValidation(inside_new_region);
                    // 由于未设置额外的结束条件，会执行搜索直到不存在满足条件的点为止
                    helper_expander_.expand();
                    // 判断满足条件的点的数量
                    if (need_cover > 10)
                    {
                        // 数量条件满足，认为找到了新的可覆盖区域
                        std::cout << ">>>>>>>>>>>>>>>>> Find ReCover Region, Size " << need_cover << " ID " << _gen_region_id(new_region) << " <<<<<<<<<<<<<<<<<" <<std::endl;
                        return true;
                    }
                }
            }
            else
            {
                if (!cover_.isGlobalCleaned(x, y))
                {
                    return true;
                }
            }
            return false;
        };

        path_.clear();
        // 复位已覆盖区域标记
        cleaned_region_.clear();
        bool should_continue = true;
        while(should_continue)
        {
            // 复位结束条件、每个点的处理函数、可行性附加判断函数为nullptr
            expander_.reset();
            cover_.reset();

            // 找到从当前位置到区域边界的路径

            // 设置点可行性判断条件为当前区域内的点
            expander_.setPoseValidation(boost::bind(&RegionManager::isInsideCurrentRegion, this, _1, _2));
            // 设置结束条件为到达当前区域的边界
            expander_.setShouldTerminate(boost::bind(reach_bound, _1, _2, _3));
            // 开始以机器人当前位置为起点的广度优先搜索
            auto res = expander_.expand();
            if (res)
            {
                // 如果能到达边界，获取去往边界的路径
                auto start_to_bound_path = expander_.getPath();
                path_.insert(path_.end(), start_to_bound_path.begin(), start_to_bound_path.end());
                env_ptr_->drawPath(start_to_bound_path);
            }
            else
            {
                // 如果不能到达边界，将起点加入
                environment::PathNode pn{0};
                pn.b = 255;
                pn.x = start_x_;
                pn.y = start_y_;
                path_.emplace_back(pn);
            }

            // 找到当前区域的可达边界，并整理为以前面搜得的最近点为起点的沿边路径，被障碍物隔断的边界通过搜索得到的路径连接
            environment::Path bound_path{0};
            bound_path = getCurrentRegionEdge();
            path_.insert(path_.end(), bound_path.begin(), bound_path.end());

            // 标记已经沿着边界清扫过的位置
            cover_.markPathCleaned(bound_path);
            // 更新规划起点为前面路径的末尾点
            cover_.setStart(path_.back().x, path_.back().y);
            // 规划覆盖路径
            cover_.planning();
            auto cover_path = cover_.getPath();
            path_.insert(path_.end(), cover_path.begin(), cover_path.end());
            // 标记覆盖区域
            cover_.markPathCleaned(cover_path);
            // 在这里显示边界路径，避免在标记已清扫区域时被覆盖
            env_ptr_->drawPath(bound_path);
            // 显示覆盖路径
            env_ptr_->drawPath(cover_path);

            // 标记当前区域已被覆盖
            cleaned_region_[_gen_region_id(*current_region_)] = true;

            expander_.reset();
            // 设置新的结束条件为发现覆盖条件的新区域
            expander_.setShouldTerminate(boost::bind(reach_new_region, _1, _2, _3));
            // 将起点设置为上次覆盖结束的位置
            expander_.setStart(path_.back().x, path_.back().y);
            should_continue = expander_.expand();

            // 如果找到了新区域
            if (should_continue)
            {
                // 获取到达新区域的路径
                auto to_region_path = expander_.getPath();
                path_.insert(path_.end(), to_region_path.begin(), to_region_path.end());
                // 标记沿着此路径的区域已经被清扫
                cover_.markPathCleaned(to_region_path);
                // 设置当前区域为新区域
                setCurrentRegion(&getRegionById(to_region_path.back().x, to_region_path.back().y));
                env_ptr_->drawPath(to_region_path);
                // 更新起点为新区域中的点
                expander_.setStart(to_region_path.back().x, to_region_path.back().y);
            }
        }

        return true;
    }

    void RegionManager::setCurrentRegion(Region *r)
    {
        current_region_ = r;
    }

    RegionManager::Region *RegionManager::getCurrentRegion()
    {
        if (current_region_ == nullptr)
        {
            addRegion(start_x_, start_y_);
        }
        return current_region_;
    }

    bool RegionManager::isInsideCurrentRegion(int x, int y)
    {
        return _is_inside(x, y, current_region_);
    }

    environment::Path &RegionManager::getPath()
    {
//        return cover_.getPath();
        return path_;
    }

    RegionManager::Region &RegionManager::addRegion(int x, int y)
    {
        Region r;
        int x_normalize = x / region_size_;
        int y_normalize = y / region_size_;

        r.xh = x_normalize * region_size_ + region_size_;
        r.xl = x_normalize * region_size_;
        r.yh = y_normalize * region_size_ + region_size_;
        r.yl = y_normalize * region_size_;

        auto id = _gen_region_id(r);
        if (regions_.find(id) == regions_.end())
        {
            regions_[id] = r;
        }

        return regions_[id];
    }

    RegionManager::RegionManager(environment::EnvironmentInterfacePtr &env, std::string name)
    {
        initialize(env, std::move(name));
    }

    RegionManager::Region &RegionManager::getRegionById(int x, int y)
    {
        for (auto &reg : regions_)
        {
            auto& r = reg.second;
            if (_is_inside(x, y, &r))
            {
                return r;
            }
        }

        return addRegion(x, y);
    }

    void RegionManager::setStart(int x, int y)
    {
        env_ptr_->toGridAndInsideGrid(x, y, x, y);
        start_x_ = x;
        start_y_ = y;

        setCurrentRegion(&getRegionById(x, y));
        expander_.setStart(x, y);
        cover_.setStart(x, y);
    }

    void RegionManager::showCurrentRegion()
    {
        showRegion(*getCurrentRegion());
    }

    void RegionManager::showRegion(const RegionManager::Region &r)
    {
        env_ptr_->drawLine(r.xl, r.yl, r.xl, r.yh);
        env_ptr_->drawLine(r.xl, r.yl, r.xh, r.yl);
        env_ptr_->drawLine(r.xh, r.yh, r.xl, r.yh);
        env_ptr_->drawLine(r.xh, r.yh, r.xh, r.yl);
    }

    const RegionManager::Region &RegionManager::generateHigherRegion(const RegionManager::Region &cur_region)
    {
        Region res;
        res.xh = cur_region.xh;
        res.xl = cur_region.xl;
        res.yh = cur_region.yl;
        res.yl = cur_region.yl - region_size_;
        _gen_region_id(res);
        regions_[res.id] = res;
        return regions_[res.id];
    }

    const RegionManager::Region &RegionManager::generateLowerRegion(const RegionManager::Region &cur_region)
    {
        Region res;
        res.xh = cur_region.xh;
        res.xl = cur_region.xl;
        res.yh = cur_region.yh + region_size_;
        res.yl = cur_region.yl;
        _gen_region_id(res);
        regions_[res.id] = res;
        return regions_[res.id];
    }

    const RegionManager::Region &RegionManager::generateLeftRegion(const RegionManager::Region &cur_region)
    {
        Region res;
        res.xh = cur_region.xl;
        res.xl = cur_region.xl - region_size_;
        res.yh = cur_region.yh;
        res.yl = cur_region.yl;
        _gen_region_id(res);
        regions_[res.id] = res;
        return regions_[res.id];
    }

    const RegionManager::Region &RegionManager::generateRightRegion(const RegionManager::Region &cur_region)
    {
        Region res;
        res.xh = cur_region.xh + region_size_;
        res.xl = cur_region.xh;
        res.yh = cur_region.yh;
        res.yl = cur_region.yl;
        _gen_region_id(res);
        regions_[res.id] = res;
        return regions_[res.id];
    }

    void RegionManager::setGoal(int x, int y)
    {

    }

    std::string RegionManager::_gen_region_id(const RegionManager::Region& reg)
    {
        std::string res;
        res = std::to_string(reg.xl) + ':' + std::to_string(reg.xh) + ':' +
              std::to_string(reg.yl) + ':' + std::to_string(reg.yh);
        return res;
    }

    std::string RegionManager::_gen_point_id(int x, int y)
    {
        std::string res;
        res = std::to_string(x) + ':' + std::to_string(y);
        return res;
    }

    bool RegionManager::_is_inside(int x, int y, RegionManager::Region *r)
    {
        if (r == nullptr)
        {
            return false;
        }
        if (x < r->xh && y < r->yh && x >= r->xl && y >= r->yl)
        {
            return true;
        }
        return false;
    }

    bool RegionManager::_is_boundary(int x, int y, const Region &r) const
    {
        if (x == r.xl || y == r.yl || x == (r.xh-1) || y == (r.yh-1) ||
            x == env_ptr_->getGridXSizeInCells()-1 || y == env_ptr_->getGridYSizeInCells()-1)
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

    environment::Path RegionManager::getCurrentRegionEdge()
    {
        environment::Path res{0};
        std::unordered_map<std::string, bool> feasible_point{0};
        bool first_boundary_point = true;
        environment::GridPoint first_point{0};
        expander_.reset();
        expander_.setPoseValidation(boost::bind(&RegionManager::isInsideCurrentRegion, this, _1, _2));
        // 标记边界点
        std::function<bool(int, int, unsigned char, const Region &)> mark_boundary = [&](int x, int y, unsigned char cost, const Region& r)->bool
        {
            if (!_is_boundary(x, y, r) )
            {
                return true;
            }

            if (first_boundary_point)
            {
                first_boundary_point = false;
                first_point.x = x;
                first_point.y = y;
                env_ptr_->setIntGridValueByGridXY(x, y, 255, 0, 0);
            }
            else
            {
                env_ptr_->setIntGridValueByGridXY(x, y, 100, 100, 100);
            }

            auto pid = _gen_point_id(x, y);
            feasible_point[pid] = true;
            return true;
        };

        // 设置每步动作
        expander_.setStepProcess(boost::bind(mark_boundary, _1, _2, _3, *getCurrentRegion()));
        // 启动一次区域内点遍历
        expander_.expand();

        std::vector<environment::Path> paths;
        environment::Path path;
        environment::PathNode pn{0};
        int first_point_index{-1}, first_path_index{-1};
        // 收集边界点并组成一段段路径
        auto get_path = [&](int x, int y)
        {
            if (feasible_point[_gen_point_id(x, y)])
            {
                if (x == first_point.x && y == first_point.y)
                {
                    first_path_index = paths.size();
                    first_point_index = path.size();
                }
                path.emplace_back(pn);
            }
            else
            {
                if (path.empty())
                {
                    return;
                }

                paths.emplace_back(path);
                path.clear();
            }
        };
        pn.r = 128;
        pn.g = 0;
        pn.b = 128;

        // 按逆时针顺序提取可通行边界
        path.clear();
        for (int i = expander_.getMaxX(); i >= expander_.getMinX(); i--)
        {
            pn.x = i;
            pn.y = expander_.getMinY();
            get_path(pn.x, pn.y);
        }
        if (!path.empty())
        {
            paths.emplace_back(path);
            path.clear();
        }

        for (int i = expander_.getMinY(); i < expander_.getMaxY(); i++)
        {
            pn.x = expander_.getMinX();
            pn.y = i;
            get_path(pn.x, pn.y);
        }
        if (!path.empty())
        {
            paths.emplace_back(path);
            path.clear();
        }

        for (int i = expander_.getMinX(); i < expander_.getMaxX(); i++)
        {
            pn.x = i;
            pn.y = expander_.getMaxY();
            get_path(pn.x, pn.y);
        }
        if (!path.empty())
        {
            paths.emplace_back(path);
            path.clear();
        }

        for (int i = expander_.getMaxY(); i >= expander_.getMinY(); i--)
        {
            pn.x = expander_.getMaxX();
            pn.y = i;
            get_path(pn.x, pn.y);
        }
        if (!path.empty())
        {
            paths.emplace_back(path);
            path.clear();
        }


        // 将一段段路径按逆时针顺序进行拼接
        environment::Path temp_path, bound_path;
        int cut_index{-1};
        for (int i = 0; i < paths.size(); i++)
        {
            if (i != first_path_index)
            {
                temp_path.insert(temp_path.end(), paths[i].begin(), paths[i].end());
            }
            else
            {
                // 找出第一个点的索引
                for (auto p : paths[i])
                {
                    if (p.x == first_point.x && p.y == first_point.y)
                    {
                        cut_index = temp_path.size();
                    }
                    temp_path.emplace_back(p);
                }
            }
        }

        if (temp_path.empty())
        {
            return temp_path;
        }
        // 显示首先搜索到的边界点
        env_ptr_->setIntGridValueByGridXY(temp_path[cut_index].x, temp_path[cut_index].y, 255, 0, 0);

        // 调整顺序，得出一条以第一个搜索到的点为起点的路径
        for (int i = cut_index; i < temp_path.size(); i++)
        {
            bound_path.emplace_back(temp_path[i]);
        }
        for (int i = 0; i < cut_index; i++)
        {
            bound_path.emplace_back(temp_path[i]);
        }

        std::function<bool(int,int,unsigned char,int,int)> goal_reach = [&](int x, int y, unsigned char cost, int goal_x, int goal_y)->bool
        {
            if (x == goal_x && y == goal_y)
            {
                return true;
            }
            return false;
        };


        auto connect_to_point = [&](environment::PathNode p1, environment::PathNode p2)
        {
            if (abs(p1.x - p2.x) > 1 || abs(p1.y - p2.y) > 1)
            {
                expander_.setStart(p1.x, p1.y);
                expander_.setShouldTerminate(boost::bind(goal_reach, _1, _2, _3, p2.x, p2.y));
                expander_.expand();
                auto res = expander_.getPath();
                bound_path.insert(bound_path.end(), res.begin(), res.end());
            }
            else
            {
                bound_path.emplace_back(p1);
            }
        };

        temp_path = bound_path;
        bound_path.clear();
        // 规划路径连接路径中的断点
        for (int i = 0; i < temp_path.size() - 1; i++)
        {
            const auto &p1 = temp_path[i];
            const auto &p2 = temp_path[i+1];
            connect_to_point(p1, p2);
        }
//        const auto &p1 = temp_path.back();
//        const auto &p2 = temp_path.front();
//        connect_to_point(p1, p2);

        return bound_path;
    }
}
