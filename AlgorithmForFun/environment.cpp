#include "environment.h"

namespace environment
{
    void
    Environment::initialize(int length, int width, int display_scale)
    {
        rect_size_ = display_scale;
        width_ = width;
        length_ = length;
        img_length_ = length_ * rect_size_, img_width_ = width_ * rect_size_;
        _initialize_grid();

        cv::namedWindow("InteractiveWindow");
        cv::namedWindow("PlanningGrid");

        initialized_ = true;
    }

    void
    Environment::display()
    {
        if (!initialized_)
        {
            throw std::runtime_error("Should initialize first.");
        }
        imshow("InteractiveWindow", display_img_);
        imshow("PlanningGrid", planning_grid_);
    }

    uint8_t
    Environment::getGridValueFromDisp(int x, int y)
    {
        if (!initialized_)
        {
            throw std::runtime_error("Should initialize first.");
        }
        int tx = (x / rect_size_), ty = (y / rect_size_);

        return getGridValue(tx, ty);
    }


    uint8_t Environment::getGridValue(int x, int y)
    {
        if (!insideGrid(x, y))
        {
            throw std::runtime_error("Out of bound.");
        }
        return *planning_grid_.col(x).row(y).data;
    }

    bool
    Environment::setGridValueFromDisp(int x, int y, uint8_t value)
    {
        if (!initialized_)
        {
            std::cerr << "Should initialize first." << std::endl;
            return false;
        }
        int tx = (x / rect_size_), ty = (y / rect_size_);

        return setGridValue(tx, ty, value);
    }

    bool Environment::setGridValue(int x, int y, uint8_t value)
    {
        if (!insideGrid(x, y))
        {
            return false;
        }

        circle(planning_grid_, cv::Point(x, y), 0,
               cv::Scalar(value), 0);

        return true;
    }

    bool Environment::setInteractiveGridValue(int x, int y, uint8_t r, uint8_t g, uint8_t b)
    {
        if (x >= img_width_ || x < 0 || y >= img_length_ || y < 0)
        {
            return false;
        }

        _normalize_xy(x, y, x, y);

        rectangle(display_img_, cv::Rect(x, y, rect_size_, rect_size_),
                  cv::Scalar(b, g, r), -1);

        return true;
    }

    void Environment::drawPath(const Path &path)
    {
        int x1, y1, x2, y2;
        int half_rect_ = rect_size_ / 2;
        for (int i = 0; i < path.size() - 1; i++)
        {
            auto &n1 = path[i];
            auto &n2 = path[i+1];
            x1 = n1.x * rect_size_ + half_rect_;
            y1 = n1.y * rect_size_ + half_rect_;
            x2 = n2.x * rect_size_ + half_rect_;
            y2 = n2.y * rect_size_ + half_rect_;
            line(display_img_, cv::Point(x1, y1), cv::Point(x2, y2),
                 cv::Scalar(n1.b, n1.g, n1.r, n1.a),1,  cv::LINE_8, 0);
        }
    }

    int Environment::getGridXSizeInCells()
    {
        return width_;
    }

    int Environment::getGridYSizeInCells()
    {
        return length_;
    }

    bool Environment::insideGridFromDisp(int x, int y)
    {
        if (!initialized_)
        {
            throw std::runtime_error("Should initialize first.");
        }
        int tx = (x / rect_size_), ty = (y / rect_size_);
        if (tx >= width_ || tx < 0 || ty >= length_ || ty < 0)
        {
            return false;
        }
        return true;
    }

    bool Environment::insideGrid(int x, int y)
    {
        if (!initialized_)
        {
            throw std::runtime_error("Should initialize first.");
        }
        if (x >= width_ || x < 0 || y >= length_ || y < 0)
        {
            return false;
        }
        return true;
    }

    int Environment::getScale()
    {
        return rect_size_;
    }

    bool Environment::displayXY2PlanningXY(int raw_x, int raw_y, int &x, int &y)
    {
        x = (raw_x / rect_size_), y = (raw_y / rect_size_);
        if (!insideGrid(x, y))
        {
            return false;
        }
        return true;
    }

    void Environment::_initialize_grid()
    {
        display_img_ = cv::Mat(img_length_, img_width_, CV_8UC3, cv::Scalar(255, 255, 255));
        planning_grid_ = cv::Mat(length_, width_, CV_8UC1, cv::Scalar(255, 255, 255));
    }

    void Environment::markStart(int x, int y, int r, int g, int b)
    {
        setInteractiveGridValue(start_x_, start_y_, 255, 255, 255);
        start_x_ = x;
        start_y_ = y;
        start_r_ = r;
        start_g_ = g;
        start_b_ = b;
        setInteractiveGridValue(start_x_, start_y_, r, g, b);
        have_start_ = true;
    }

    void Environment::markGoal(int x, int y, int r, int g, int b)
    {
        setInteractiveGridValue(goal_x_, goal_y_, 255, 255, 255);
        goal_x_ = x;
        goal_y_ = y;
        goal_r_ = r;
        goal_g_ = g;
        goal_b_ = b;
        setInteractiveGridValue(goal_x_, goal_y_, r, g, b);
        have_goal_ = true;
    }

    void Environment::markObstacle(int x, int y)
    {
        obstacles_.insert(std::make_tuple(x, y));
        setInteractiveGridValue(x, y, 0, 0, 0);
    }

    void Environment::_normalize_xy(int x, int y, int &nx, int &ny)
    {
        nx = (x / rect_size_) * rect_size_;
        ny = (y / rect_size_) * rect_size_;
    }

    void Environment::showStartGoalPose()
    {
        if (have_start_)
        {
            setInteractiveGridValue(start_x_, start_y_, start_r_, start_g_, start_b_);
        }
        if (have_goal_)
        {
            setInteractiveGridValue(goal_x_, goal_y_, goal_r_, goal_g_, goal_b_);
        }
    }

    bool Environment::planningXY2InteractiveXY(int ix, int iy, int &ox, int &oy)
    {
        ox = ix * rect_size_;
        oy = iy * rect_size_;

        return true;
    }

    bool Environment::setIntGridValByPlanXY(int x, int y, uint8_t r, uint8_t g, uint8_t b)
    {
        auto ix = x * rect_size_;
        auto iy = y * rect_size_;

        return setInteractiveGridValue(ix, iy, r, g, b);
    }

    void Environment::clear()
    {
        display_img_ = cv::Mat(img_length_, img_width_, CV_8UC3, cv::Scalar(255, 255, 255));
        showStartGoalPose();
        for (auto &o : obstacles_)
        {
            setInteractiveGridValue(std::get<0>(o), std::get<1>(o), 0, 0, 0);
        }
    }

    void Environment::reset()
    {
        _initialize_grid();
        obstacles_.clear();
    }

}