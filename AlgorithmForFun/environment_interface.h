#ifndef DFS_ENVIRONMENT_INTERFACE_H
#define DFS_ENVIRONMENT_INTERFACE_H

#include <iostream>
#include <chrono>
#include <thread>
#include <tuple>
#include <unordered_map>

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

    struct Footprint
    {
        Point center;
        std::vector<Point> footprint;
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


//        struct Edge
//        {
//            float xIntersect, yIntersect, yUpper, yLoser, xUpper, xLower;
//            float dxPerScan;
//            Edge *next;
//        };
//
//        void
//        insertEdge(Edge *list, Edge *edge)
//        {
//            Edge *p, *q = list;
//            p = q->next;
//            while(p != nullptr)
//            {
//                if (edge->xIntersect < p->xIntersect)
//                {
//                    p = nullptr;
//                }
//                else
//                {
//                    q = p;
//                    p = p->next;
//                }
//            }
//            edge->next = q->next;
//            q->next = edge;
//        }
//
//        int
//        yNext(int k, int cnt, GridPoint *pts)
//        {
//            int j;
//            if ((k+1) > (cnt-1))
//            {
//                j = 0;
//            }
//            else
//            {
//                j = k + 1;
//            }
//            while(pts[k].y == pts[j].y)
//            {
//                if ((j+1) > (cnt-1))
//                {
//                    j = 0;
//                }
//                else
//                {
//                    j++;
//                }
//            }
//            return pts[j].y;
//        }
//
//        void
//        makeEdgeRec(GridPoint lower, GridPoint upper, int yComp, Edge *edge, Edge *edges[])
//        {
//            edge->dxPerScan = (float)(upper.x - lower.x) / (upper.y - lower.y);
//            edge->xIntersect = (float)lower.x;
//            if (upper.y < yComp)
//            {
//                edge->yUpper = upper.y - 1;
//            }
//            else
//            {
//                edge->yUpper = upper.y;
//            }
//            insertEdge(edges[(int)lower.y], edge);
//        }
//
//        void
//        buildActiveList(int scan, Edge *active, Edge *edges[])
//        {
//            Edge *p, *q;
//            p = edges[scan]->next;
//            while(p)
//            {
//                q = p->next;
//                insertEdge(active, p);
//                p = q;
//            }
//        }
//
//        void
//        fillscan(int scan, Edge *active)
//        {
//            Edge *p1, *p2;
//            int i;
//            p1 = active->next;
//            while(p1)
//            {
//                p2 = p1->next;
//                for (i = (int)p1->xIntersect; i < (int)p2->xIntersect; i++)
//                {
////                    PutPixel((int)i, scan, 0);
//                    setGridValue((int)i, scan, 0);
//                    setInteractiveGridValue((int)i * getScale(), scan * getScale(), 0);
//                }
//                p1 = p2->next;
//            }
//        }
//
//        void
//        deleteAfter(Edge *q)
//        {
//            Edge *p = q->next;
//            q->next = p->next;
//            free(p);
//        }
//
//        void
//        updateActiveList(int scan, Edge *active)
//        {
//            Edge *q = active, *p = active->next;
//            while(p)
//            {
//                if (scan >= p->yUpper)
//                {
//                    p = p->next;
//                    deleteAfter(q);
//                }
//                else
//                {
//                    p->xIntersect = p->xIntersect + p->dxPerScan;
//                    q = p;
//                    p = p->next;
//                }
//            }
//        }
//
//        void
//        resortActiveList(Edge *active)
//        {
//            Edge *q, *p = active->next;
//            active->next = nullptr;
//            while(p)
//            {
//                q = p->next;
//                insertEdge(active, p);
//                p = q;
//            }
//        }
//
//        void
//        buildEdgeList(int cnt, GridPoint *pts, Edge *edges[])
//        {
//            Edge *edge;
//            GridPoint v1{}, v2{};
//            int yPrev = pts[cnt-2].y;
//            v1.x = pts[cnt-1].x;
//            v1.y = pts[cnt-1].y;
//            for (int i = 0; i < cnt; i++)
//            {
//                v2 = pts[i];
//                if (v1.y != v2.y)
//                {
//                    edge = (Edge*)malloc(sizeof(Edge));
//                    if (v1.y < v2.y)
//                    {
//                        makeEdgeRec(v1, v2, yNext(i, cnt, pts), edge, edges);
//                    }
//                    else
//                    {
//                        makeEdgeRec(v2, v1, yPrev, edge, edges);
//                    }
//                    yPrev = v1.y;
//                    v1 = v2;
//                }
//            }
//        }
//
//        void
//        scanFill(int cnt, GridPoint *pts)
//        {
//            int win_h = 100;
//            Edge *edges[win_h], *active;
//            int scan;
//            for (int i = 0; i < win_h; i++)
//            {
//                edges[i] = (Edge *)malloc(sizeof(Edge));
//                edges[i]->next = nullptr;
//            }
//            buildEdgeList(cnt, pts, edges);
//
//            active = (Edge *)malloc(sizeof (edges));
//            active->next = nullptr;
//            for (scan = 0; scan < win_h; scan++)
//            {
//                buildActiveList(scan, active, edges);
//                if (active->next)
//                {
//                    fillscan(scan, active);
//                    updateActiveList(scan, active);
//                    resortActiveList(active);
//                }
//            }
//        }


        struct Edge
        {
            int ymax, x;
            float inv_k;
            std::shared_ptr<Edge> next;
        };

        std::unordered_map<int, std::shared_ptr<Edge>> et_;

        void
        _create_et(std::vector<GridPoint>& points)
        {
            auto proc_lin = [&](GridPoint &ph, GridPoint &pl,
                                bool perpendicular)
            {
//                std::cout << ph->x <<"  " <<ph->y <<std::endl;
                auto &e = et_[ph.y];
                while(e)
                {
                    e = e->next;
                }
                e = std::make_shared<Edge>();
                e->ymax = ph.y;
                if (perpendicular)
                {
                    e->inv_k = std::numeric_limits<float>::infinity();
                }
                else
                {
                    e->inv_k = (float)(ph.x - pl.x) / (float)(ph.y - pl.y);
                }
                e->x = pl.x;
                std::cout << e->ymax << " " << e->x << " " << e->inv_k <<std::endl;
            };

            auto get_ph_pl = [&](GridPoint p1, GridPoint p2, GridPoint &ph, GridPoint &pl, bool &perpend)
            {
//                std::cout << p1.x <<"  " <<p1.y <<std::endl;
                perpend = false;
                if (p1.y > p2.y)
                {
                    ph = p1;
                    pl = p2;
                }
                else if (p1.y < p2.y)
                {
                    ph = p2;
                    pl = p1;
                }
                else
                {
                    ph = p2;
                    pl = p1;
                    perpend = true;
                }
//                std::cout << ph->x <<"  " <<ph->y <<std::endl;
            };

            GridPoint ph, pl;
            bool perpendicular{false};
            for (int i = 0; i < points.size() - 1; i++)
            {
//                std::cout << points[i].x << " " << points[i].y <<std::endl;
                get_ph_pl(points[i], points[i+1], ph, pl, perpendicular);
//                std::cout << ph.x <<"  " <<ph.y <<std::endl;
                proc_lin(ph, pl, perpendicular);
            }
            get_ph_pl(points.back(), points.front(), ph, pl, perpendicular);
            proc_lin(ph, pl, perpendicular);
        }

        void
        _ael_process(std::shared_ptr<Edge>& ael)
        {
            auto p = ael;
            std::unordered_map<int, std::shared_ptr<Edge>> xsorted;
            while(p)
            {
                xsorted[p->x] = p;
                p = p->next;
            }
            for (auto &tp : xsorted)
            {
                std::cout << tp.second->x << std::endl;
            }
            std::cout << std::endl;
        }

        void
        _loop()
        {
            for (auto &e : et_)
            {
                _ael_process(et_[e.second->ymax]);
            }
        }

        void
        fillPolygon(std::vector<GridPoint>& polygon)
        {
            _create_et(polygon);
            _loop();
        }


    };

    using EnvironmentInterfacePtr = std::shared_ptr<EnvironmentInterface>;
}
#endif //DFS_ENVIRONMENT_INTERFACE_H
