#include "region_manager.h"

namespace algorithm
{
    void RegionManager::initialize(environment::EnvironmentInterfacePtr &env, std::string name)
    {
        env_ptr_ = env;
        name_ = name;
        expander_.initialize(env, name + "Expander");
//        expander_.setShouldTerminate(boost::bind(&RegionManager::_is_boundary, this, _1, _2));
        cover_.initialize(env, name + "Cover");
        cover_.setPoseValidation(boost::bind(&RegionManager::isInsideCurrentRegion, this, _1, _2));

        side_points_.emplace_back(boost::bind(&RegionManager::_get_right, this, _1, _2, _3, _4, _5, 1));
        side_points_.emplace_back(boost::bind(&RegionManager::_get_higher_right, this, _1, _2, _3, _4, _5, 1));
        side_points_.emplace_back(boost::bind(&RegionManager::_get_middle_higher, this, _1, _2, _3, _4, _5, 1));
        side_points_.emplace_back(boost::bind(&RegionManager::_get_higher_left, this, _1, _2, _3, _4, _5, 1));
        side_points_.emplace_back(boost::bind(&RegionManager::_get_left, this, _1, _2, _3, _4, _5, 1));
        side_points_.emplace_back(boost::bind(&RegionManager::_get_lower_left, this, _1, _2, _3, _4, _5, 1));
        side_points_.emplace_back(boost::bind(&RegionManager::_get_middle_lower, this, _1, _2, _3, _4, _5, 1));
        side_points_.emplace_back(boost::bind(&RegionManager::_get_lower_right, this, _1, _2, _3, _4, _5, 1));

        initialized_ = true;
    }

    bool RegionManager::planning()
    {
        std::function<bool(int, int, unsigned char)> reach_bound =
                [&](int x, int y, unsigned char cost)->bool
        {
            return _is_boundary(x, y, *current_region_);
        };

        std::function<bool(int, int, unsigned char)> reach_new_region =
                [&](int x, int y, unsigned char cost)->bool
        {
            if (!_is_inside(x, y, current_region_))
            {
                auto new_region = getRegionById(x, y);
                if (!cleaned_region_[_gen_region_id(new_region)])
                {
                    return true;
                }
            }
            return false;
        };

        path_.clear();
        cleaned_region_.clear();
        bool should_continue = true;
        while(should_continue)
        {
            FL_PRINT
            expander_.reset();
            expander_.setPoseValidation(boost::bind(&RegionManager::isInsideCurrentRegion, this, _1, _2));
            expander_.setShouldTerminate(boost::bind(reach_bound, _1, _2, _3));
            expander_.expand();
            auto start_to_bound_path = expander_.getPath();
            path_.insert(path_.end(), start_to_bound_path.begin(), start_to_bound_path.end());

            FL_PRINT
            auto bound_path = getCurrentRegionEdge();
            path_.insert(path_.end(), bound_path.begin(), bound_path.end());

            FL_PRINT
            cover_.reset();
            FL_PRINT
            cover_.markPathCleaned(path_);
            FL_PRINT
            cover_.setStart(start_to_bound_path.back().x, start_to_bound_path.back().y);
            FL_PRINT
            cover_.planning();
            FL_PRINT
            auto cover_path = cover_.getPath();
            path_.insert(path_.end(), cover_path.begin(), cover_path.end());

            FL_PRINT
            cleaned_region_[_gen_region_id(*current_region_)] = true;
FL_PRINT

            expander_.reset();
            expander_.setShouldTerminate(boost::bind(reach_new_region, _1, _2, _3));
            // 将起点设置为上次覆盖结束的位置
            expander_.setStart(path_.back().x, path_.back().y);
            should_continue = expander_.expand();
            auto to_region_path = expander_.getPath();
            path_.insert(path_.end(), to_region_path.begin(), to_region_path.end());

            if (should_continue)
            {
                setCurrentRegion(&getRegionById(to_region_path.back().x, to_region_path.back().y));
                env_ptr_->drawPath(expander_.getPath());
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
            FL_PRINT
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
        env_ptr_->displayXY2PlanningXY(x, y, x, y);
        start_x_ = x;
        start_y_ = y;

        setCurrentRegion(&getRegionById(x, y));
//        showCurrentRegion();
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
            if (!_is_boundary(x, y, r) || 255 - cost > 40)
            {
                return true;
            }

            if (first_boundary_point)
            {
                first_boundary_point = false;
                first_point.x = x;
                first_point.y = y;
//                env_ptr_->setIntGridValByPlanXY(x, y, 255, 0, 0);
            }
            else
            {
//                env_ptr_->setIntGridValByPlanXY(x, y, 100, 100, 100);
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
        env_ptr_->setIntGridValByPlanXY(temp_path[cut_index].x, temp_path[cut_index].y, 255, 0, 0);

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

        temp_path = bound_path;
        int temp_cnt = 0;
        // 规划路径连接路径中的断点
        for (int i = 0; i < temp_path.size() - 1; i++)
        {
            const auto &p1 = temp_path[i];
            const auto &p2 = temp_path[i+1];
            if (abs(p1.x - p2.x) > 1 || abs(p1.y - p2.y) > 1)
            {
                FL_PRINT
                expander_.setStart(p1.x, p1.y);
                expander_.setShouldTerminate(boost::bind(goal_reach, _1, _2, _3, p2.x, p2.y));
                expander_.expand();
                auto res = expander_.getPath();
                bound_path.insert(bound_path.begin()+i+temp_cnt+1, res.begin(), res.end());
                temp_cnt += res.size();
            }
        }

        FL_PRINT
//        path_.insert(path_.end(), bound_path.begin(), bound_path.end());
//        env_ptr_->drawPath(bound_path);
        return bound_path;
    }

}