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

        RegionManager(environment::EnvironmentInterfacePtr &env, std::string name);

        void initialize(environment::EnvironmentInterfacePtr &env, std::string name) override;

        bool planning() override;

        environment::Path &getPath() override;

        /**
         * @return true 生成并插入了新Region
         * @return false 已存在包含此点的Region
         * */
        bool addRegion(int x, int y);

        const Region& getCurrentRegion(int x, int y);

        void setGoal(int x, int y) override;;

        void setStart(int x, int y) override;

        void showCurrentRegion(int x, int y);

        void showRegion(const Region& r);

        const Region& generateHigherRegion(const Region& cur_region);

        const Region& generateLowerRegion(const Region& cur_region);

        const Region& generateLeftRegion(const Region& cur_region);

        const Region& generateRightRegion(const Region& cur_region);

    private:

        bool initialized_{false};
        environment::EnvironmentInterfacePtr env_ptr_;
        environment::GridPoint region_origin_;
        int region_size_{100};
        int start_x_{0}, start_y_{0};
        std::unordered_map<std::string, Region> regions_;
        Region *current_region_;

        std::vector<std::function<bool(environment::EnvironmentInterfacePtr&, int, int, int&, int&)>> side_points_;
        environment::Path path_;
        Region empty_region_{0};

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
