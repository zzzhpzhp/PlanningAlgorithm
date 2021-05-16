#ifndef AFF_ENVIRONMENT_H
#define AFF_ENVIRONMENT_H

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
        markObstacle(int x, int y) override;

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
        std::shared_ptr<Edge> ael_;
        std::vector<std::shared_ptr<Edge>> x_sorted_;
        std::unordered_map<int, std::shared_ptr<Edge>> et_;

        void
        _create_et(const GridPolygon& points)
        {
            auto create_edge = [&](const GridPoint *ph, const GridPoint *pl, bool perpendicular)
            {
                y_min_ = std::min(pl->y, y_min_);
                y_max_ = std::max(ph->y, y_max_);

                std::shared_ptr<Edge> cur;
                // 找到链表尾部
                if (!et_[pl->y])
                {
                    // 如果当前桶为空，则初始化当前桶
                    et_[pl->y] = std::make_shared<Edge>();
                    // 指向当前边
                    cur = et_[pl->y];
                }
                else
                {
                    // 当前桶不为空，找到当前链表的尾部，新建边，并指向新建边
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

                // 初始化当前边的数据结构
                cur->ymax = (float)ph->y;
                if (perpendicular)
                {
                    cur->inv_k = std::numeric_limits<float>::infinity();
                }
                else
                {
                    cur->inv_k = (float)(ph->x - pl->x) / (float)(ph->y - pl->y);
                }
                cur->x = (float)pl->x;
            };

            auto get_ph_pl = [&](const GridPoint *p1, const GridPoint *p2,
                                 const GridPoint* &ph, const GridPoint* &pl, bool &perpend)
            {
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
            };

            const GridPoint *ph{}, *pl{};
            bool perpendicular{false};
            for (int i = 0; i < points.size() - 1; i++)
            {
                get_ph_pl(&points[i], &points[i+1], ph, pl, perpendicular);
                create_edge(ph, pl, perpendicular);
            }
            get_ph_pl(&points.back(), &points.front(), ph, pl, perpendicular);
            create_edge(ph, pl, perpendicular);
        }

        void
        _sort_ael()
        {
            std::shared_ptr<Edge> dummy;
            dummy = std::make_shared<Edge>();
            dummy->next = std::make_shared<Edge>();
            std::shared_ptr<Edge> pre = dummy, cur = dummy->next;

            auto p = ael_->next;
            if (p)
            {
                cur->x = p->x;
                cur->ymax = p->ymax;
                cur->inv_k = p->inv_k;

                p = p->next;
            }
            else
            {
                return;
            }

            while (p)
            {
                pre = dummy;
                cur = dummy->next;

                bool ok = false;
                while(cur)
                {
                    if (p->x < cur->x)
                    {
                        auto new_edge = std::make_shared<Edge>();

                        new_edge->x = p->x;
                        new_edge->ymax = p->ymax;
                        new_edge->inv_k = p->inv_k;

                        pre->next = new_edge;
                        new_edge->next = cur;
                        ok = true;
                        break;
                    }
                    pre = cur;
                    cur = cur->next;
                }

                if (!ok)
                {
                    cur = std::make_shared<Edge>();
                    pre->next = cur;

                    cur->x = p->x;
                    cur->ymax = p->ymax;
                    cur->inv_k = p->inv_k;
                    cur->next = nullptr;
                }

                p = p->next;
            }

            ael_ = dummy;
        }

        void
        _insert_ael(std::shared_ptr<Edge> &cur)
        {
            while(cur)
            {
                auto pre = ael_;
                auto p = ael_->next;
                // 将当前y对应的边加入
                bool ok = false;
                while (p)
                {
                    if (cur->x <= p->x)
                    {
                        // 前节点的后节点更新为新节点
                        pre->next = cur;
                        // cur节点更新为其下一个节点
                        cur = cur->next;
                        // 续接被打断的节点
                        pre->next->next = p;
                        // 此次排序完成
                        ok = true;
                        break;
                    }

                    pre = p;
                    p = p->next;
                }

                if (!ok)
                {
                    pre->next = cur;
                    cur = cur->next;
                    pre->next->next = nullptr;
                }
            }
        }

        void
        _ael_update(int y)
        {
            if (!ael_)
            {
                ael_ = std::make_shared<Edge>();
            }

            // 删除ymax小于当前y的边
            auto pre = ael_;
            auto p = ael_->next;
            while (p)
            {
                if (y >= p->ymax)
                {
                    pre->next = p->next;
                    p = pre->next;
                    continue;
                }
                // 修改x，加上1/k
                p->x = p->x + p->inv_k;
                pre = p;
                p = p->next;
            }

            _sort_ael();

            // 读取与y相关的边
            auto cur = et_[y];

            if (!cur)
            {
                return;
            }

            _insert_ael(cur);
        }

        void
        _fill_line(int y)
        {
            int x1, x2;
            auto p1 = ael_->next;
            if (!p1)
            {
                return ;
            }
            auto p2 = p1->next;

            while (p2)
            {
                if (p1->x >= p2->x)
                {
                    x2 = (int)p1->x;
                    x1 = (int)p2->x;
                }
                else
                {
                    x1 = (int)p1->x;
                    x2 = (int)p2->x;
                }

                for (int i = x1; i <= x2; i++)
                {
                    setInteractiveGridValue(i * rect_size_, y * rect_size_, 0);
                    setGridValue((int)i, y, 0);
                }

                p1 = p2->next;
                if (!p1)
                {
                    break;
                }
                p2 = p2->next->next;
            }
        }

        void
        _bresenham_line(int x1, int y1, int x2, int y2)
        {
            int dx, dy, x, y, d, d1, d2, inc, tmp;
            int cur_x, cur_y;

            dx = x2 - x1;
            dy = y2 - y1;

            if (dx * dy >= 0)
            {
                inc = 1;
            }
            else
            {
                inc = -1;
            }

            d1 = 2 * dy - dx;

            while (dx == 0 &&  dy == 0)
            {
                if (d1 >= 0)
                {
                    d2 = d1 + 2 * (dy - dx);
                    y += 1;
                }
                else
                {
                    d2 = d1 + 2 * dy;
                }

                d1 = d2;

                cur_x = x1 + x;
                cur_y = y1 + y;
                setInteractiveGridValue(cur_x * rect_size_, cur_y * rect_size_, 0);
                setGridValue(cur_x, cur_y, 0);
                std::cout << "x " << cur_x << " cur_y " << cur_y << std::endl;

                x += 1;
                dx = x2 - cur_x;
                dy = y2 - cur_y;
            }


        }

    };
}
#endif //AFF_ENVIRONMENT_H
