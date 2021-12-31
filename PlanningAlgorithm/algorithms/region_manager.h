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

#include <region_manager_expander.h>

namespace algorithm
{
    class RegionManager : public AlgorithmInterface
    {
    public:
        Expander expander_;
        using Region = struct
        {
            int xl, xh, yl, yh;
            std::string id;
        };

        RegionManager(environment::EnvironmentInterfacePtr &env, std::string name);

        void initialize(environment::EnvironmentInterfacePtr &env, std::string name) override;

        bool planning() override;

        environment::Path &getPath() override;

        /**
         * @return true 生成并插入了新Region
         * @return false 已存在包含此点的Region
         * */
        Region& addRegion(int x, int y);

        Region& getRegionById(int x, int y);

        void setGoal(int x, int y) override;;

        void setStart(int x, int y) override;

        void showCurrentRegion();

        void showRegion(const Region& r);

        void setCurrentRegion(Region *r);

        Region *getCurrentRegion()
        {
            return current_region_;
        }

        const Region& generateHigherRegion(const Region& cur_region);

        const Region& generateLowerRegion(const Region& cur_region);

        const Region& generateLeftRegion(const Region& cur_region);

        const Region& generateRightRegion(const Region& cur_region);

    private:

        bool initialized_{false};
        environment::EnvironmentInterfacePtr env_ptr_;
        environment::GridPoint region_origin_{};
        int region_size_{100};
        int start_x_{0}, start_y_{0};
        std::unordered_map<std::string, Region> regions_;
        Region *current_region_{nullptr};

        std::vector<std::function<bool(environment::EnvironmentInterfacePtr&, int, int, int&, int&)>> side_points_;
        environment::Path path_;
        Region empty_region_{0};
        std::unordered_map<std::string, bool> is_reachable_boundary_;

    private:

        bool _is_boundary(int x, int y) const;

        std::string _get_region_id(int x, int y);

        static std::string _gen_id_for_point(int x, int y);

        static std::string _gen_id_for_region(const Region& reg);

        int _update_boundary_reachability()
        {

        }
    };
}
