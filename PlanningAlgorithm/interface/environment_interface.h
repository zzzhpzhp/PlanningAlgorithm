#pragma once

#include <iostream>
#include <chrono>
#include <thread>
#include <tuple>
#include <unordered_map>
#include <queue>
#include <atomic>

#include <boost/bind.hpp>

#include<opencv2/core/core.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>

#include "typedefine.h"
#define FL_PRINT {std::cout << __FUNCTION__ << " " << __LINE__ << std::endl;};
namespace environment
{
    const int NO_INFORMATION = 150;
    const int LETHAL_OBSTACLE = 0;
    const int INSCRIBED_INFLATED_OBSTACLE = 50;
    const int PENALTY_COST = 100;
    const int FREE_SPACE = 255;
    enum MarkMode {MARK_OBSTACLE, MARK_FREE_SPACE};
    class EnvironmentInterface
    {
    public:
        virtual void
        initialize(int length, int width, int display_scale) = 0;

        virtual uint8_t
        getGridValueFromDisp(int x, int y) = 0;

        virtual uint8_t
        getGridValue(int x, int y) = 0;

        virtual bool
        setGridValueFromDisp(int x, int y, uint8_t value) = 0;

        virtual bool
        setGridValue(int x, int y, uint8_t value) = 0;

        virtual int
        getRobotRadius()
        {
            return robot_radius_;
        }

        virtual void
        setRobotRadius(int robot_radius) = 0;

        virtual void
        setAlgorithmRunningDelayTime(float second)
        {
            delay_time_ = second;
        }

        virtual void
        setDisplayDelayTime(float second)
        {
            display_delay_time_ = second;
        }

        virtual float
        getDisplayDelayTime()
        {
            return display_delay_time_;
        }

        virtual double
        getAlgorithmRunningDelayTime()
        {
            return delay_time_;
        }

        virtual bool
        saveEnvironmntToDisk(std::string path) = 0;

        virtual bool
        loadEnvironmentFromDisk(std::string path) = 0;

        /**
         * @brief 交互界面中设置一个网格的颜色
         * */
        virtual bool
        setInteractiveGridValue(int x, int y, uint8_t r = 0, uint8_t g = 0, uint8_t b = 0, uint8_t a = 0) = 0;

        virtual bool
        setIntGridValByPlanXY(int x, int y, uint8_t r = 0, uint8_t g = 0, uint8_t b = 0, uint8_t a = 70) = 0;

        virtual void
        drawPath(const Path &path) = 0;

        virtual int
        getGridXSizeInCells() = 0;

        virtual int
        getGridYSizeInCells() = 0;

        virtual bool
        insideGrid(int x, int y) = 0;

        virtual bool
        insideGridFromDisp(int x, int y) = 0;

        virtual int
        getScale() = 0;

        virtual void
        display() = 0;

        virtual bool
        displayXY2PlanningXY(int raw_x, int raw_y, int &x, int &y) = 0;

        virtual bool
        planningXY2InteractiveXY(int ix, int iy, int &ox, int &oy) = 0;

        virtual void
        clear() = 0;

        virtual void
        reset() = 0;

        virtual void
        start()
        {
            is_running_.store(true);
        }

        virtual void
        stop()
        {
            is_running_.store(false);
        }

        virtual bool
        isRunning()
        {
            return is_running_.load();
        }

        virtual void
        markStart(int x, int y, int r = 0, int g = 0, int b = 0) = 0;

        virtual void
        markGoal(int x, int y, int r = 0, int g = 0, int b = 0) = 0;

        virtual void
        markObstacle(int x, int y, int obstacle_radius = 1) = 0;

        virtual void
        markFreeSpace(int x, int y, int radius = 1) = 0;

        virtual void
        showStartGoalPose() = 0;

        virtual std::tuple<int, int>
        getStart() = 0;

        virtual std::tuple<int, int>
        getGoal() = 0;

        virtual void
        play(Path &path) = 0;

        virtual void
        setFootprint(Footprint footprint) = 0;

        virtual const Footprint&
        getFootprint() = 0;

        virtual bool
        generateFootprintByRadius() = 0;

        virtual void
        drawPolygon(const GridPolygon &polygon) = 0;

        virtual void
        fillGridPolygon(const GridPolygon& polygon) = 0;

        virtual void
        drawGridLine(int x1, int y1, int x2, int y2) = 0;

        virtual void
        drawLine(int x1, int y1, int x2, int y2) = 0;

        virtual void
        fillPolygonOutline(const GridPolygon& polygon) = 0;

        virtual Grids
        getPolygonPoints(const GridPolygon &polygon) = 0;

        virtual Grids
        getPolygonOutlinePoints(const GridPolygon &polygon) = 0;

        virtual void
        fillPoints(const Grids& points) = 0;

        virtual void
        switchMarkMode()
        {
            mark_mode_ = (mark_mode_ + 1) % 2;
            switch (mark_mode_.load())
            {
                case MARK_OBSTACLE:
                    std::cout << "Current mark mode is MARK_OBSTACLE" << std::endl;
                    break;
                case MARK_FREE_SPACE:
                    std::cout << "Current mark mode is MARK_FREE_SPACE" << std::endl;
                    break;
                default:
                    std::cerr << "Unknow mark mode." << std::endl;
                    break;
            }
        }

        virtual int
        getMarkMode()
        {
            return mark_mode_;
        };

        virtual int
        getCurrentAlgorithmIndex()
        {
            return algorithm_index_.load();
        }

        virtual void
        setCurrentAlgorithmIndex(int index)
        {
            algorithm_index_.store(index);
        }

        virtual void
        worldToMap(float wx, float wy, int &mx, int &my)
        {
            if (wx >= 0)
            {
                mx = (int)(wx / resolution_.load() + 0.05f);
            }
            else
            {
                mx = (int)(wx / resolution_.load() - 0.05f);
            }

            if (wy >= 0)
            {
                my = (int)(wy / resolution_.load() + 0.05f);
            }
            else
            {
                my = (int)(wy / resolution_.load() - 0.05f);
            }
        }

        virtual void
        mapToWorld(int mx, int my, float &wx, float &wy)
        {
            wx = (float)mx * resolution_.load();
            wy = (float)my * resolution_.load();
        }

        virtual unsigned char
        getCost(int x, int y) {};

        virtual void
        setCost(int x, int y, unsigned char cost) {};

        virtual unsigned char
        getCost(float x, float y)
        {
            int mx, my;
            worldToMap(x, y, mx, my);
            return getCost(mx, my);
        }

        virtual void
        setCost(float x, float y, unsigned char cost)
        {
            int mx, my;
            worldToMap(x, y, mx, my);
            setCost(mx, my, cost);
        }

    protected:

        std::atomic<int> robot_radius_{5}, algorithm_index_{0};
        std::atomic<float> delay_time_{0.001};
        std::atomic<float> display_delay_time_{0.001};
        std::atomic_bool is_running_{false};
        std::atomic<int> mark_mode_{MARK_OBSTACLE};
        std::atomic<float> resolution_{0.05f};
    };

    using EnvironmentInterfacePtr = std::shared_ptr<EnvironmentInterface>;
}
