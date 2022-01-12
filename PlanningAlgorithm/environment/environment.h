#pragma once
#include <iostream>
#include <chrono>
#include <thread>
#include <mutex>
#include <fstream>

#include <boost/unordered_map.hpp>
#include <boost/bind.hpp>

#include<opencv2/core/core.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>

#include "environment_interface.h"
#include <jsoncpp/json/json.h>
#define cvMoveWindow cv::moveWindow
namespace environment
{
    class Environment : public EnvironmentInterface
    {
    public:
        const std::string ENVIRONMENT_VERSION{"V1.3"};

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

        float
        getResolution() override
        {
            return resolution_;
        }

        bool
        setGridValue(int x, int y, uint8_t value) override;

        void
        toGrid(int x, int y, int &outx, int &outy) override;

        void
        fromGrid(int x, int y, int &outx, int &outy) override;

        bool
        setInteractiveGridValue(int x, int y, uint8_t r = 0, uint8_t g = 0, uint8_t b = 0, uint8_t a = 255) override;

        bool
        setIntGridValueByGridXY(int x, int y, uint8_t r = 0, uint8_t g = 0, uint8_t b = 0, uint8_t a = 255) override;

        bool
        setIntGridValByPlanXY(int x, int y, uint8_t r, uint8_t g, uint8_t b, uint8_t a) override;

        void
        drawPath(const Path &path) override;

        int
        getGridXSizeInCells() override;

        int
        getGridYSizeInCells() override;

        bool
        saveEnvironmntToDisk(std::string path);

        bool
        loadEnvironmentFromDisk(std::string path);

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
        markFreeSpace(int x, int y, int radius = 1) override;

        void
        showStartGoalPose() override;

        std::tuple<int, int>
        getStart() override;

        std::tuple<int, int>
        getGoal() override;

        unsigned char
        getCost(int x, int y) override
        {
            return getGridValueFromDisp(x, y);
        }

        void
        play(Path &path) override;

        void
        setFootprint(Footprint footprint) override;

        void
        setFootprintCost(int x, int y, unsigned char cost) override;

        void
        setRobotRadius(int robot_radius) override;

        const Footprint&
        getFootprint() override;

        bool
        generateFootprintByRadius() override;

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

        void
        drawCircleFromDisp(int x, int y, int radius, int line_width, int r = 0, int g = 0, int b = 0, int a = 255) override
        {
            std::lock_guard<std::mutex> lg(display_img_mtx_);
//            CVAPI(void)  cvCircle( CvArr* img, CvPoint center, int radius,
//                                   CvScalar color, int thickness CV_DEFAULT(1),
//                                   int line_type CV_DEFAULT(8), int shift CV_DEFAULT(0));
            cv::circle(display_img_, cv::Point(x, y ), radius,
                 cv::Scalar(0, 0, 0, 0), line_width, 0);
        }

    private:

        Footprint footprint_;
        bool initialized_{false};
        bool have_start_ = false, have_goal_ = false;

        int rect_size_{5};
        int length_{200}, width_{200};
        int img_length_ = length_ * rect_size_, img_width_ = width_ * rect_size_;
        int start_x_{0}, start_y_{0};
        int goal_x_{0}, goal_y_{0};
        int start_r_, start_g_, start_b_ = 255;
        int goal_r_ = 255, goal_g_, goal_b_;

        std::mutex display_img_mtx_, planning_grid_mtx_;

        using Obstacle = std::tuple<int, int, unsigned char>;
        struct Comp
        {
            template<typename T>
            bool operator()(const T& l, const T& r) const
            {
                if (std::get<0>(l) == std::get<0>(r))
                {
                    if (std::get<1>(l) == std::get<1>(r))
                    {
                        return false;
                    }
                    return std::get<1>(l) > std::get<1>(r);
                }
                return std::get<0>(l) > std::get<0>(r);
            }
        };
        std::set<Obstacle, Comp> obstacles_;

        static bool edge_cmp(const std::shared_ptr<Edge> &a, const std::shared_ptr<Edge> &b)
        {
            return a->x < b->x;
        };
        int y_min_, y_max_;
        int obstacle_radius_{0}, free_brush_radius_{0}, cost_radius_{10};
        std::shared_ptr<Edge> ael_;
        std::vector<std::shared_ptr<Edge>> x_sorted_;
        std::unordered_map<int, std::shared_ptr<Edge>> et_;
        std::vector<GridPoint> cost_area_, free_brush_area_;

        float cost_scaling_factor = 0.5;

        cv::Mat display_img_, planning_grid_;
        // di: display image pi: planning image
        cv::Mat di_pp_, di_pn_, di_nn_, di_np_, pi_pp_, pi_pn_, pi_nn_, pi_np_;

        cv::Mat *
        _get_planning_container(int x, int y);

        cv::Mat *
        _get_display_container(int x, int y);

        void
        _initialize_grid();

        void
        _normalize_xy(int x, int y, int &nx, int &ny);

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
