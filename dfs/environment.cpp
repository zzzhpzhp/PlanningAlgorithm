#include "environment.h"

namespace planner
{
    void
    Environment::initialize(int length, int width, int display_scale)
    {
        rect_size_ = display_scale;
        width_ = width;
        length_ = length;
        img_length_ = length_ * rect_size_, img_width_ = width_ * rect_size_;
        display_img_ = cv::Mat(img_length_, img_width_, CV_8UC1, cv::Scalar(255, 255, 255));
        planning_grid_ = cv::Mat(length, width, CV_8UC1, cv::Scalar(255, 255, 255));
    }

    uint8_t
    Environment::getGridValue(int x, int y)
    {
        int tx = (x / rect_size_), ty = (y / rect_size_);
        if (tx > width_ || tx < 0 || ty > length_ || ty < 0)
        {
            throw std::runtime_error("Out of bound.");
        }
//        std::cout << (int)*planning_grid_.col(tx).row(ty).data << " " << (int)*planning_grid_.col(tx+1).row(ty+1).data << "\n";
        return *planning_grid_.col(tx).row(ty).data;
    }

    void
    Environment::display()
    {
        imshow("disp_img", display_img_);
        imshow("planning_grid", planning_grid_);
        cv::waitKey(1);
    }

    bool
    Environment::setGridValue(int x, int y, uint8_t value)
    {
        int tx = (x / rect_size_), ty = (y / rect_size_);
        if (tx >= width_ || tx < 0 || ty >= length_ || ty < 0)
        {
            return false;
        }

        circle(planning_grid_, cv::Point(tx, ty), 0,
               cv::Scalar(0, 0, 255), 0);

        // 将点放大为方块显示
        int disp_x = tx * rect_size_, disp_y = ty * rect_size_;
        rectangle(display_img_, cv::Rect(disp_x, disp_y, rect_size_, rect_size_),
                  cv::Scalar(value, 0, 0, 0), -1);
    }

    int Environment::getGridXSizeInCells()
    {
        return width_;
    }

    int Environment::getGridYSizeInCells()
    {
        return length_;
    }

    bool Environment::insideGrid(int x, int y)
    {
        int tx = (x / rect_size_), ty = (y / rect_size_);
        if (tx >= width_ || tx < 0 || ty >= length_ || ty < 0)
        {
            return false;
        }
        return true;
    }

    int Environment::getScale()
    {
        return rect_size_;
    }

}