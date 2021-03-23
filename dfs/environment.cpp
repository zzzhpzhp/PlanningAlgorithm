#include "environment.h"

namespace planner
{
    void
    Environment::initialize(int length, int width, int display_scale)
    {
        //创建Mat图像（像素值自定义）
        rect_size_ = display_scale;
        width_ = width;
        length_ = length;
        img_length_ = length_ * rect_size_, img_width_ = width_ * rect_size_;
        display_img_ = cv::Mat(img_length_, img_width_, CV_8UC1, cv::Scalar(255, 255, 255));
        planning_grid_ = cv::Mat(length, width, CV_8UC1, cv::Scalar(255, 255, 255));
    }

    cv::Mat&
    Environment::getPlanningObj()
    {
        return planning_grid_;
    }

    cv::Mat&
    Environment::getDisplayObj()
    {

        return display_img_;
    }

    uint8_t
    Environment::getGridValue(int x, int y)
    {
        if (x > width_ || x < 0 || y > length_ || y < 0)
        {
            throw std::runtime_error("Out of bound.");
        }
//        return *(planning_grid_.data + y + x * width_);
        return *planning_grid_.col(x).row(y).data;
//        return planning_grid_.at<uint8_t>(x, y);
    }

    bool
    Environment::setGridValue(int x, int y, uint8_t value)
    {
        if (x >= width_ || x < 0 || y >= length_ || y < 0)
        {
            return false;
        }

        circle(planning_grid_, cv::Point(x, y), 0,
               cv::Scalar(0, 0, 255), 0);

        // 将点放大为方块显示
        int disp_x = x * rect_size_, disp_y = y * rect_size_;
        rectangle(display_img_, cv::Rect(disp_x, disp_y, rect_size_, rect_size_),
                  cv::Scalar(value, 0, 0, 0), -1);
    }

    void
    Environment::eventCallback(int event, int x, int y, int flags, void *)
    {
        if ((event == CV_EVENT_LBUTTONDOWN) || (flags & CV_EVENT_FLAG_LBUTTON))
        {
            int tx = (x / rect_size_), ty = (y / rect_size_);
            if (tx >= width_ || tx < 0 || ty >= length_ || ty < 0)
            {
                return;
            }
            setGridValue(tx, ty, 0);

            std::cout << (int)getGridValue(tx, ty) << "\n";
        }
    }
}