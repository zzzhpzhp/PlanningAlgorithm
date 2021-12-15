#pragma once
#include <iostream>
#include <chrono>
#include <thread>
#include <mutex>

#include <boost/unordered_map.hpp>
#include <boost/bind.hpp>

#include<opencv2/core/core.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>

#include "environment_interface.h"

namespace environment
{

    class Environment : public EnvironmentInterface
    {
    public:
        void
        initialize(int length, int width, int display_scale) override;

        void
        display() override;

        uint8_t
        getGridValueFromDisp(int x, int y) override;

        uint8_t
        getGridValue(int x, int y) override;

        bool
        setGridValueFromDisp(int x, int y, uint8_t value) override;

        bool
        setGridValue(int x, int y, uint8_t value) override;

        bool
        setInteractiveGridValue(int x, int y, uint8_t r = 0, uint8_t g = 0, uint8_t b = 0, uint8_t a = 255) override;

        bool
        setIntGridValByPlanXY(int x, int y, uint8_t r, uint8_t g, uint8_t b) override;

        void
        drawPath(const Path &path) override;

        int
        getGridXSizeInCells() override;

        int
        getGridYSizeInCells() override;

        bool
        insideGrid(int x, int y) override;

        bool
        insideGridFromDisp(int x, int y) override;

        int
        getScale() override;

        bool
        displayXY2PlanningXY(int raw_x, int raw_y, int &x, int &y) override;

        bool
        planningXY2InteractiveXY(int ix, int iy, int &ox, int &oy) override;

        void
        clear() override;

        void
        reset() override;

        void
        markStart(int x, int y, int r, int g, int b) override;

        void
        markGoal(int x, int y, int r, int g, int b) override;

        void
        markObstacle(int x, int y, int obstacle_radius = 1) override;

        void
        showStartGoalPose() override;

        std::tuple<int, int>
        getStart() override;

        std::tuple<int, int>
        getGoal() override;

        void
        play(Path &path) override;

        void
        setFootprint(Footprint footprint) override;

        void
        drawPolygon(const GridPolygon &polygon) override;

        void
        drawLine(int x1, int y1, int x2, int y2) override;

        void
        fillGridPolygon(const GridPolygon& polygon) override;

        void
        drawGridLine(int x1, int y1, int x2, int y2) override;

        void
        fillPolygonOutline(const GridPolygon& polygon) override;

        Grids
        getPolygonPoints(const GridPolygon &polygon) override;

        Grids
        getPolygonOutlinePoints(const GridPolygon &polygon) override;

        void
        fillPoints(const Grids& points) override;

    private:

        Footprint footprint_;
        bool initialized_{false};
        bool have_start_ = false, have_goal_ = false;

        cv::Mat display_img_, planning_grid_;

        int rect_size_{5};
        int length_{200}, width_{200};
        int img_length_ = length_ * rect_size_, img_width_ = width_ * rect_size_;
        int start_x_{0}, start_y_{0};
        int goal_x_{0}, goal_y_{0};
        int start_r_, start_g_, start_b_ = 255;
        int goal_r_ = 255, goal_g_, goal_b_;

        std::mutex display_img_mtx_, planning_grid_mtx_;

        using Obstacle = std::tuple<int, int>;
        std::set<Obstacle> obstacles_;

        void
        _initialize_grid();

        void
        _normalize_xy(int x, int y, int &nx, int &ny);

        static bool edge_cmp(const std::shared_ptr<Edge> &a, const std::shared_ptr<Edge> &b)
        {
            return a->x < b->x;
        };
        int y_min_, y_max_;
        int obstacle_radius_{0};
        std::shared_ptr<Edge> ael_;
        std::vector<std::shared_ptr<Edge>> x_sorted_;
        std::vector<GridPoint> catched_obstacle_area_;
        std::unordered_map<int, std::shared_ptr<Edge>> et_;

        void
        _set_cost(int x, int y);

        void
        _get_points(int x, int y, std::shared_ptr<Grids> &points);

        void
        _create_et(const GridPolygon& points);

        void
        _sort_ael();

        void
        _insert_ael(std::shared_ptr<Edge> &cur);

        void
        _ael_update(int y);

        void
        _fill_line(std::function<void(int, int)> action, int y);

        void
        _bresenham_line(std::function<void(int, int)> action, int x1, int y1, int x2, int y2);

    };
}
