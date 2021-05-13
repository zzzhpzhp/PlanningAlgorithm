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

            bool operator()(const std::shared_ptr<Edge> &a, const std::shared_ptr<Edge> &b) const
            {
                return a->x > b->x;
            };
        };

        std::unordered_map<int, std::shared_ptr<Edge>> et_;
        int y_min_, y_max_;

        void
        _create_et(std::vector<GridPoint>& points)
        {
            auto create_edge = [&](GridPoint *ph, GridPoint *pl, bool perpendicular)
            {
//                std::cout << ph->x <<"  " <<ph->y <<std::endl;
                y_min_ = std::min(pl->y, y_min_);
                y_max_ = std::max(ph->y, y_max_);

                std::shared_ptr<Edge> cur;
                // 找到链表尾部
                if (!et_[pl->y])
                {
                    et_[pl->y] = std::make_shared<Edge>();
                    cur = et_[pl->y];
                }
                else
                {
                    cur = et_[pl->y];
                    auto pre = cur;
                    while(cur)
                    {
                        pre = cur;
                        cur = cur->next;
                    }
                    pre->next = std::make_shared<Edge>();
                    cur = pre->next;
                }

                cur->ymax = ph->y;
                if (perpendicular)
                {
                    cur->inv_k = std::numeric_limits<float>::infinity();
                }
                else
                {
                    cur->inv_k = (float)(ph->x - pl->x) / (float)(ph->y - pl->y);
                }
                cur->x = pl->x;
//                std::cout << cur->ymax << " " << cur->x << " " << cur->inv_k <<std::endl;
            };

            auto get_ph_pl = [&](GridPoint *p1, GridPoint *p2, GridPoint* &ph, GridPoint* &pl, bool &perpend)
            {
//                std::cout << "p1 xy " << p1->x <<"  " <<p1->y <<std::endl;
//                std::cout << "p2 xy " << p2->x <<"  " <<p2->y <<std::endl;
                perpend = false;
                if (p1->y > p2->y)
                {
                    ph = p1;
                    pl = p2;
                }
                else if (p1->y < p2->y)
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
//                std::cout << "pl xy " << pl->x <<"  " <<pl->y <<std::endl;
//                std::cout << "ph xy " << ph->x <<"  " <<ph->y <<std::endl;
            };

            GridPoint *ph{}, *pl{};
            bool perpendicular{false};
            for (int i = 0; i < points.size() - 1; i++)
            {
//                std::cout << points[i].x << " " << points[i].y <<std::endl;
                get_ph_pl(&points[i], &points[i+1], ph, pl, perpendicular);
//                std::cout << ph->x <<"  " <<ph->y <<std::endl;
                create_edge(ph, pl, perpendicular);
            }
            get_ph_pl(&points.back(), &points.front(), ph, pl, perpendicular);
            create_edge(ph, pl, perpendicular);
        }

        void
        _ael_process(int y)
        {
            auto ael = et_[y];
//            std::cout << " y " << y << std::endl;
            if (!ael)
            {
                std::cout << "empty ael" << std::endl;
                return;
            }
            std::cout << "have " << y << std::endl;
            auto p = ael;
            std::priority_queue<std::shared_ptr<Edge>, std::vector<std::shared_ptr<Edge>>, Edge> xsorted;
            while(p)
            {
//                std::cout << "e ymax " << p->ymax << " x " << p->x << " k_inv " << p->inv_k << std::endl;
                xsorted.push(p);
                p = p->next;
            }
            for (int i = 0; i < xsorted.size(); i++)
            {
                auto top = xsorted.top();
                xsorted.pop();
//                std::cout << top->x << " ";
            }
//            for (auto &tp : xsorted)
//            {
//                std::cout << tp.second->x << " ";
//            }
            std::cout << std::endl;
        }

        void
        _loop()
        {
            auto &e = et_[0];
            while(e)
            {
                std::cout << e->ymax << " " << e->x << " " << e->inv_k <<std::endl;
                e = e->next;
            }

//            for (int i = y_min_; i <= y_max_; i++)
//            {
//                _ael_process(i);
//            }
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
