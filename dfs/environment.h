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

namespace planner
{
    class Environment
    {
    public:
        void
        initialize(int length, int width, int display_scale);

        cv::Mat&
        getPlanningObj();

        cv::Mat&
        getDisplayObj();

        void
        eventCallback(int event, int x, int y, int flags, void *);

        uint8_t
        getGridValue(int x, int y);

        bool
        setGridValue(int x, int y, uint8_t value);

    private:
        cv::Mat display_img_, planning_grid_;
        int rect_size_ = 5;
        int length_ = 200, width_ = 200;
        int img_length_ = length_ * rect_size_, img_width_ = width_ * rect_size_;
    };
}
#endif //DFS_ENVIRONMENT_H
