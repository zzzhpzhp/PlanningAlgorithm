#ifndef DFS_ENVIRONMENT_INTERFACE_H
#define DFS_ENVIRONMENT_INTERFACE_H

#include <iostream>
#include <chrono>
#include <thread>
#include <tuple>
#include <unordered_map>
#include <queue>

#include <boost/bind.hpp>

#include<opencv2/core/core.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>

namespace environment
{
    struct PathNode
    {
        int x, y;
        uint8_t r, g, b, a;
    };

    using Path = std::vector<PathNode>;

    struct Point
    {
        float x, y;
    };

    struct GridPoint
    {
        int x, y;
    };

    struct Pose
    {
        float x, y, theta;
    };

    using Polygon = std::vector<Point>;
    using GridPolygon = std::vector<GridPoint>;
    using Grids = std::vector<GridPoint>;

    struct Footprint
    {
        Point center;
        std::vector<Point> footprint;
    };

    struct Edge
    {
        float ymax{}, x{};
        float inv_k{};
        std::shared_ptr<Edge> next;

        bool operator()(const std::shared_ptr<Edge> &a, const std::shared_ptr<Edge> &b) const
        {
            return a->x > b->x;
        };
    };

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

        virtual bool
        setInteractiveGridValue(int x, int y, uint8_t r = 0, uint8_t g = 0, uint8_t b = 0, uint8_t a = 0) = 0;

        virtual bool
        setIntGridValByPlanXY(int x, int y, uint8_t r = 0, uint8_t g = 0, uint8_t b = 0) = 0;

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
        markStart(int x, int y, int r = 0, int g = 0, int b = 0) = 0;

        virtual void
        markGoal(int x, int y, int r = 0, int g = 0, int b = 0) = 0;

        virtual void
        markObstacle(int x, int y) = 0;

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
    };

    using EnvironmentInterfacePtr = std::shared_ptr<EnvironmentInterface>;
}
#endif //DFS_ENVIRONMENT_INTERFACE_H
