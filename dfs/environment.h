#ifndef DFS_ENVIRONMENT_H
#define DFS_ENVIRONMENT_H

#include <iostream>
#include <chrono>
#include <thread>

#include <boost/unordered_map.hpp>
#include <boost/bind.hpp>

#include<opencv2/core/core.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>

#include "environment_interface.h"

namespace planner
{
    class Environment : public EnvironmentInterface
    {
    public:
        void
        initialize(int length, int width, int display_scale) override;

        void
        display() override;

        uint8_t
        getGridValue(int x, int y) override;

        bool
        setGridValue(int x, int y, uint8_t value) override;

        int
        getGridXSizeInCells() override;

        int
        getGridYSizeInCells() override;

        bool
        insideGrid(int x, int y) override;

        int
        getScale() override;

    private:
        cv::Mat display_img_, planning_grid_;
        int rect_size_ = 5;
        int length_ = 200, width_ = 200;
        int img_length_ = length_ * rect_size_, img_width_ = width_ * rect_size_;

    };
}
#endif //DFS_ENVIRONMENT_H
