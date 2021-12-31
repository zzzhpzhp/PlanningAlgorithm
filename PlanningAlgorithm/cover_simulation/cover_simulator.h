//#pragma once
//
//#include <stack>
//#include <deque>
//#include <queue>
//#include <chrono>
//#include <thread>
//#include <iostream>
//#include <unordered_map>
//
//#include <boost/bind.hpp>
//
//#include "algorithm_interface.h"
//#include "environment_interface.h"
//
//#include <region_manager.h>
//
//namespace algorithm
//{
//    class CoverSimulator : public AlgorithmInterface
//    {
//    public:
//        CoverSimulator(environment::EnvironmentInterfacePtr &env, std::string name)
//        {
//            initialize(env, std::move(name));
//        }
//
//        std::tuple<int, int, int, int> getNextRegion();
//
//
//    private:
//
//        environment::EnvironmentInterfacePtr env_ptr_;
//        environment::GridPoint region_origin_;
//        int region_size_{(int)(4.0f / env_ptr_->getResolution())};
//
//    private:
//
//
//
//    };
//}
