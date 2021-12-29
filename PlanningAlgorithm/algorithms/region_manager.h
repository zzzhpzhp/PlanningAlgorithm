#pragma once

#include <stack>
#include <deque>
#include <queue>
#include <chrono>
#include <thread>
#include <iostream>
#include <set>
#include <unordered_map>

#include <boost/bind.hpp>

#include "algorithm_interface.h"
#include "environment_interface.h"

namespace algorithm
{
    class RegionManager : public AlgorithmInterface
    {
    public:
        using Region = struct
        {
            int xl, xh, yl, yh;
            std::string id;
        };

        RegionManager(environment::EnvironmentInterfacePtr &env, std::string name)
        {
            initialize(env, std::move(name));
        }

        void initialize(environment::EnvironmentInterfacePtr &env, std::string name)
        {
            env_ptr_ = env;
            name_ = name;

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

        std::tuple<int, int, int, int> getNextRegion();

        bool planning();

        environment::Path &getPath();

        /**
         * @return true 生成并插入了新Region
         * @return false 已存在包含此点的Region
         * */
        bool addRegion(int x, int y);

        const Region& getCurrentRegion(int x, int y)
        {
            for (const auto &reg : regions_)
            {
                const auto& r = reg.second;
                if (x < r.xh && y < r.yh && x >= r.xl && y >= r.yl)
                {
                    return r;
                }
            }

            return Region{0};
        }

        void
        setGoal(int x, int y) {};

        void
        setStart(int x, int y) override
        {
            start_x_ = x;
            start_y_ = y;
        }


        void showCurrentRegion(int x, int y)
        {
            if (_get_region_id(x, y).empty())
            {
                addRegion(x, y);
            }
            env_ptr_->drawCircle(x, y, 3, -1);
            const Region& cur_region = getCurrentRegion(x, y);
            showRegion(cur_region);
//
//            {
//                const Region& new_region = generateHigherRegion(cur_region);
//                showRegion(new_region);
//            }
//
//            {
//                const Region& new_region = generateLowerRegion(cur_region);
//                showRegion(new_region);
//            }
//
//            {
//                const Region& new_region = generateLeftRegion(cur_region);
//                showRegion(new_region);
//            }
//
//            {
//                const Region& new_region = generateRightRegion(cur_region);
//                showRegion(new_region);
//
//                auto r = generateRightRegion(new_region);
//                showRegion(r);
//                auto l = generateLeftRegion(new_region);
//                showRegion(l);
//                auto low = generateLowerRegion(new_region);
//                showRegion(low);
//                auto h = generateHigherRegion(new_region);
//                showRegion(h);
//            }
        }

        void showRegion(const Region& r)
        {
            env_ptr_->drawLine(r.xl, r.yl, r.xl, r.yh);
            env_ptr_->drawLine(r.xl, r.yl, r.xh, r.yl);
            env_ptr_->drawLine(r.xh, r.yh, r.xl, r.yh);
            env_ptr_->drawLine(r.xh, r.yh, r.xh, r.yl);
        }

        const Region& generateHigherRegion(const Region& cur_region)
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

        const Region& generateLowerRegion(const Region& cur_region)
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

        const Region& generateLeftRegion(const Region& cur_region)
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

        const Region& generateRightRegion(const Region& cur_region)
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

    private:

        bool initialized_{false};
        environment::EnvironmentInterfacePtr env_ptr_;
        environment::GridPoint region_origin_;
        int region_size_{50};
        int start_x_{0}, start_y_{0};
        struct RegionCmp
        {
            bool operator()(const Region &a, const Region &b) const
            {
                if (a.xl == b.xl && a.yl == b.yl && a.xh == b.xh && a.yh == b.yh)
                {
                    return false;
                }

                if (a.xl >= b.xl)
                {
                    return true;
                }

                if (a.yl >= b.yl)
                {
                    return true;
                }
                return false;
            }
        };
        std::unordered_map<std::string, Region> regions_;
        Region *current_region_;

        std::vector<std::function<bool(environment::EnvironmentInterfacePtr&, int, int, int&, int&)>> side_points_;
        environment::Path path_;

    private:

        bool _is_boundary(int x, int y);

        std::string _get_region_id(int x, int y);

        std::string _gen_id_for_region(Region reg)
        {
            std::string res;
            res = std::to_string(reg.xl) + ':' + std::to_string(reg.xh) + ':' +
                    std::to_string(reg.yl) + ':' + std::to_string(reg.yh);
            return res;
        }
    };
}
