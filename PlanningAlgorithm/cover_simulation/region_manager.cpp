#include "region_manager.h"

namespace algorithm
{
    void RegionManager::initialize(environment::EnvironmentInterfacePtr &env, std::string name)
    {
        env_ptr_ = env;
        name_ = name;
        expander_.initialize(env, name + "Expander");
//        expander_.setShouldTerminate(boost::bind(&RegionManager::_is_boundary, this, _1, _2));
        expander_.setPoseValidation(boost::bind(&RegionManager::isInsideCurrentRegion, this, _1, _2));
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
//        expander_.expand();
//        if (!expander_.getPath().empty())
//        {
//            env_ptr_->drawPath(expander_.getPath());
//        }
        cover_.planning();
        getCurrentRegionEdge();
    }

    void RegionManager::setCurrentRegion(Region *r)
    {
        current_region_ = r;
    }

    RegionManager::Region *RegionManager::getCurrentRegion()
    {
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

        auto id = _gen_id_for_region(r);
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
        showCurrentRegion();
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
        _gen_id_for_region(res);
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
        _gen_id_for_region(res);
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
        _gen_id_for_region(res);
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
        _gen_id_for_region(res);
        regions_[res.id] = res;
        return regions_[res.id];
    }

    void RegionManager::setGoal(int x, int y)
    {

    }

    std::string RegionManager::_gen_id_for_region(const RegionManager::Region& reg)
    {
        std::string res;
        res = std::to_string(reg.xl) + ':' + std::to_string(reg.xh) + ':' +
              std::to_string(reg.yl) + ':' + std::to_string(reg.yh);
        return res;
    }

    std::string RegionManager::_gen_id_for_point(int x, int y)
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

    bool RegionManager::_is_boundary(int x, int y) const
    {
        if (x % region_size_ == 0 || y % region_size_ == 0)
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

    void RegionManager::getCurrentRegionEdge()
    {
        std::unordered_map<std::string, bool> feasible_point;
        std::function<bool(int, int, unsigned char)> step_proc = [&](int x, int y, unsigned char cost)->bool
        {
            if (!_is_boundary(x, y))
            {
                return true;
            }

            auto pid = _gen_id_for_point(x, y);
            feasible_point[pid] = true;
            return true;
        };
        expander_.setStepProcess(boost::bind(step_proc, _1, _2, _3));
        expander_.expand();
        std::cout << "feasible_point " << feasible_point.size() << std::endl;
    }

}