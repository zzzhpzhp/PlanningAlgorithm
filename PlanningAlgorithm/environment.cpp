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
        std::lock_guard<std::mutex> dlg(display_img_mtx_);
        imshow("InteractiveWindow", display_img_);

        std::lock_guard<std::mutex> plg(planning_grid_mtx_);
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

        std::lock_guard<std::mutex> plg(planning_grid_mtx_);
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
            std::cerr << "Failed to set grid value." << std::endl;
            return false;
        }

        std::lock_guard<std::mutex> lg(planning_grid_mtx_);
        circle(planning_grid_, cv::Point(x, y), 0,
               cv::Scalar(value), 0);

        return true;
    }

    bool Environment::setInteractiveGridValue(int x, int y, uint8_t r, uint8_t g, uint8_t b, uint8_t a)
    {
        if (x >= img_width_ || x < 0 || y >= img_length_ || y < 0)
        {
            std::cerr << "Out of bound." << std::endl;
            return false;
        }

        _normalize_xy(x, y, x, y);

        std::lock_guard<std::mutex> lg(display_img_mtx_);
        rectangle(display_img_, cv::Rect(x, y, rect_size_, rect_size_),
                  cv::Scalar(b, g, r, a), -1);

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

            std::lock_guard<std::mutex> lg(display_img_mtx_);
            line(display_img_, cv::Point(x1, y1), cv::Point(x2, y2),
                 cv::Scalar(n1.b, n1.g, n1.r, n1.a),1, 8, 0);
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
        if (x >= img_width_ || x < 0 || y >= img_length_ || y < 0)
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
        if (!setInteractiveGridValue(x, y, 0, 0, 0))
        {
            return;
        }
        if (!setGridValueFromDisp(x, y, 0))
        {
            return;
        }
        obstacles_.insert(std::make_tuple(x, y));
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
        {
            std::lock_guard<std::mutex> lg(display_img_mtx_);
            display_img_ = cv::Mat(img_length_, img_width_, CV_8UC3, cv::Scalar(255, 255, 255));
        }
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

    void Environment::_normalize_xy(int x, int y, int &nx, int &ny)
    {
        nx = (x / rect_size_) * rect_size_;
        ny = (y / rect_size_) * rect_size_;
    }

    void Environment::_initialize_grid()
    {
        std::lock_guard<std::mutex> dlg(display_img_mtx_);
        display_img_ = cv::Mat(img_length_, img_width_, CV_8UC3, cv::Scalar(255, 255, 255));
        std::lock_guard<std::mutex> plg(planning_grid_mtx_);
        planning_grid_ = cv::Mat(length_, width_, CV_8UC1, cv::Scalar(255, 255, 255));
    }

    std::tuple<int, int> Environment::getStart()
    {
        return std::tuple<int, int>{start_x_, start_y_};
    }

    std::tuple<int, int> Environment::getGoal()
    {
        return std::tuple<int, int>{goal_x_, goal_y_};
    }

    void Environment::play(Path &path)
    {
        int nx, ny;
        for (const auto& n : path)
        {
            nx = n.x * rect_size_;
            ny = n.y * rect_size_;
            setInteractiveGridValue(nx, ny, n.r, n.g, n.b, 100);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }

    void
    Environment::setFootprint(Footprint footprint)
    {
        footprint_ = footprint;
    }

    void
    Environment::drawPolygon(const std::vector<GridPoint> &polygon)
    {
        int x1, y1, x2, y2;
        for (int i = 0; i < polygon.size()-1; i++)
        {
            x1 = polygon[i].x * rect_size_;
            x2 = polygon[i+1].x * rect_size_;
            y1 = polygon[i].y * rect_size_;
            y2 = polygon[i+1].y * rect_size_;
            line(display_img_, cv::Point(x1, y1), cv::Point(x2, y2),
                 cv::Scalar(0, 255, 0, 255),1,  8, 0);
        }
        x1 = polygon.back().x * rect_size_;
        y1 = polygon.back().y * rect_size_;
        x2 = polygon.front().x * rect_size_;
        y2 = polygon.front().y * rect_size_;
        line(display_img_, cv::Point(x1, y1), cv::Point(x2, y2),
             cv::Scalar(0, 255, 0, 255),1,  8, 0);
    }

    void Environment::fillGridPolygon(const std::vector<GridPoint> &polygon)
    {
        _create_et(polygon);
        for (int i = y_min_; i <= y_max_; i++)
        {
            _ael_update(i);
            _fill_line(std::bind(&Environment::_set_cost, this,
                             std::placeholders::_1, std::placeholders::_2), i);
        }
    }

    void Environment::fillPolygonOutline(const GridPolygon &polygon)
    {
        int x1{0}, y1{0}, x2{0}, y2{0};
        for (int i = 0; i < polygon.size()-1; i++)
        {
            x1 = polygon[i].x;
            x2 = polygon[i+1].x;
            y1 = polygon[i].y;
            y2 = polygon[i+1].y;

            drawGridLine(x1, y1, x2, y2);
            drawLine(x1, y1, x2, y2);
        }
        x1 = polygon.back().x;
        y1 = polygon.back().y;
        x2 = polygon.front().x;
        y2 = polygon.front().y;

        drawGridLine(x1, y1, x2, y2);
        drawLine(x1, y1, x2, y2);
    }

    Grids Environment::getPolygonOutlinePoints(const GridPolygon &polygon)
    {
        std::shared_ptr<Grids> points;
        points = std::make_shared<Grids>();
        auto fun = std::bind(&Environment::_get_points, this,
                             std::placeholders::_1, std::placeholders::_2, points);
        int x1{0}, y1{0}, x2{0}, y2{0};
        for (int i = 0; i < polygon.size()-1; i++)
        {
            x1 = polygon[i].x;
            x2 = polygon[i+1].x;
            y1 = polygon[i].y;
            y2 = polygon[i+1].y;

            _bresenham_line(fun, x1, y1, x2, y2);
        }
        x1 = polygon.back().x;
        y1 = polygon.back().y;
        x2 = polygon.front().x;
        y2 = polygon.front().y;

        _bresenham_line(fun, x1, y1, x2, y2);
        return *points;
    }

    Grids Environment::getPolygonPoints(const GridPolygon &polygon)
    {
        std::shared_ptr<Grids> points;
        points = std::make_shared<Grids>();
        _create_et(polygon);
        auto fun = std::bind(&Environment::_get_points, this,
                             std::placeholders::_1, std::placeholders::_2, points);
        for (int i = y_min_; i <= y_max_; i++)
        {
            _ael_update(i);
            _fill_line(fun, i);
        }
        return *points;
    }

    void Environment::fillPoints(const Grids& points)
    {
        for (const auto& p : points)
        {
            _set_cost(p.x, p.y);
        }
    }

    void Environment::drawGridLine(int x1, int y1, int x2, int y2)
    {
        _bresenham_line(std::bind(&Environment::_set_cost, this,
                                  std::placeholders::_1, std::placeholders::_2), x1, y1, x2, y2);
    }

    void Environment::drawLine(int x1, int y1, int x2, int y2)
    {
        line(display_img_, cv::Point(x1 * rect_size_, y1 * rect_size_),
             cv::Point(x2 * rect_size_, y2 * rect_size_),
             cv::Scalar(0, 255, 0, 255),1,  8, 0);
    }

    void Environment::_bresenham_line(std::function<void(int, int)> action, int x1, int y1, int x2, int y2)
    {
        int dx, dy, x{0}, y{0}, d1, d2;
        int cur_x, cur_y;
        int dh, dl;
        int dir, x_dir = 1, y_dir = 2;
        int *main_dir, *sub_dir, main_target, main_inc{1}, sub_inc{1};

        dx = abs(x2 - x1);
        dy = abs(y2 - y1);

        if (dx >= dy)
        {
            dir = x_dir;
            main_dir = &x;
            sub_dir = &y;
            main_target = x2 - x1;
            dh = dx;
            dl = dy;
            if (x2 < x1)
            {
                main_inc = -1;
            }
            if (y2 < y1)
            {
                sub_inc = -1;
            }
        }
        else
        {
            dir = y_dir;
            main_dir = &y;
            sub_dir = &x;
            main_target = y2 - y1;
            dh = dy;
            dl = dx;
            if (y2 < y1)
            {
                main_inc = -1;
            }
            if (x2 < x1)
            {
                sub_inc = -1;
            }
        }

        d1 = 2 * dl - dh;

        while (abs(*main_dir) < abs(main_target))
        {
            if (dir == x_dir)
            {
                cur_x = x1 + *main_dir;
                cur_y = y1 + *sub_dir;
            }
            else
            {
                cur_x = x1 + *sub_dir;
                cur_y = y1 + *main_dir;
            }

            action(cur_x, cur_y);

            if (d1 >= 0)
            {
                d2 = d1 + 2 * (dl - dh);
                *sub_dir += sub_inc;
            }
            else
            {
                d2 = d1 + 2 * dl;
            }

            d1 = d2;

            *main_dir += main_inc;
        }
    }

    void Environment::_fill_line(std::function<void(int, int)> action, int y)
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
                action(i, y);
            }

            p1 = p2->next;
            if (!p1)
            {
                break;
            }
            p2 = p2->next->next;
        }
    }

    void Environment::_ael_update(int y)
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

    void Environment::_insert_ael(std::shared_ptr<Edge> &cur)
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

    void Environment::_sort_ael()
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

    void Environment::_create_et(const GridPolygon &points)
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

    void Environment::_get_points(int x, int y, std::shared_ptr<Grids> &points)
    {
        GridPoint p{};
        p.x = x;;
        p.y = y;
        points->emplace_back(p);
    }

    void Environment::_set_cost(int x, int y)
    {
        setInteractiveGridValue(x * rect_size_, y * rect_size_, 0);
        setGridValue(x, y, 0);
    }
}
