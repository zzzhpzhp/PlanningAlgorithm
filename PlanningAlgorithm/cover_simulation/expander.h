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

using namespace cv;
namespace algorithm
{
    class Expander : public AlgorithmInterface
    {
    public:

        Expander();

        void
        initialize(environment::EnvironmentInterfacePtr &env, std::string name) override;

        void setStart(int x, int y) override;

        void setStepProcess(std::function<bool(int x, int y, unsigned char cost)> fun);

        void setPoseValidation(std::function<bool(int x, int y, unsigned char cost)> fun);

        void setShouldTerminate(std::function<bool(int x, int y, unsigned char cost)> fun);

        environment::Path &getPath() override;

        bool expand();

        void reset()
        {
            setStepProcess(nullptr);
            setPoseValidation(nullptr);
            setShouldTerminate(nullptr);
        }

        int getMaxX()
        {
            return max_x_;
        }

        int getMaxY()
        {
            return max_y_;
        }

        int getMinX()
        {
            return min_x_;
        }

        int getMinY()
        {
            return min_y_;
        }

    private:
        environment::Path path_;
        bool initialized_{false};
        int start_x_{0}, start_y_{0};
        int min_x_{0}, min_y_{0}, max_x_{0}, max_y_{0};
        std::atomic_bool is_running_{false};
        float map_cost_scale_{0.0f};
        environment::EnvironmentInterfacePtr env_ptr_;
        std::function<bool(int x, int y, unsigned char cost)> step_process_{nullptr};
        std::function<bool(int x, int y, unsigned char cost)> pose_validation_{nullptr};
        std::function<bool(int x, int y, unsigned char cost)> should_terminate_{nullptr};
        std::vector<std::function<bool(environment::EnvironmentInterfacePtr&, int, int, int&, int&)>> side_points_;

        struct Node
        {
            int x{0}, y{0};
            int id{-1};
            float dist{0.0f};
            bool in_open_list = false;
            bool in_close_list = false;
            unsigned char cost{0};
            Node *parent_node{nullptr};

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
    };
}
