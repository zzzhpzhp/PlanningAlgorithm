#include "environment.h"
#include <utility.h>

namespace environment
{
    void
    Environment::initialize(int length, int width, int display_scale)
    {
        rect_size_ = display_scale;
        length_ = length;
        width_ = width;
        img_length_ = length_ * rect_size_, img_width_ = width_ * rect_size_;
        _initialize_grid();

        cv::namedWindow("PlanningGrid");
        cvMoveWindow("PlanningGrid", img_width_+100, 0);
        cv::namedWindow("InteractiveWindow");
        cvMoveWindow("InteractiveWindow", 0, 0);

        initialized_ = true;
    }

    void
    Environment::display()
    {
        if (!initialized_)
        {
            throw std::runtime_error("Should initialize first.");
        }
        std::lock_guard<std::mutex> plg(planning_grid_mtx_);
        imshow("PlanningGrid", planning_grid_);
//        cvMoveWindow("PlanningGrid", img_width_+100, 0);
        std::lock_guard<std::mutex> dlg(display_img_mtx_);
        imshow("InteractiveWindow", display_img_);
//        cvMoveWindow("InteractiveWindow", 0, 0);
    }

    uint8_t
    Environment::getGridValueFromDisp(int x, int y)
    {
        if (!initialized_)
        {
            throw std::runtime_error("Should initialize first.");
        }
        if (!insideGridFromDisp(x, y))
        {
            return -1;
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

    void Environment::markFreeSpace(int x, int y, int radius)
    {
        if (free_brush_radius_ != radius)
        {
            GridPoint p{0};
            free_brush_area_.clear();
            free_brush_radius_ = radius;
            for (int i = -free_brush_radius_ + 1; i <= free_brush_radius_ - 1; i++)
            {
                for (int j = -free_brush_radius_ + 1; j <= free_brush_radius_ - 1; j++)
                {
                    if ((i*i + j*j) <= free_brush_radius_ * free_brush_radius_)
                    {
                        p.x = i * rect_size_;
                        p.y = j * rect_size_;
                        free_brush_area_.emplace_back(p);
                    }
                }
            }
        }

        for (auto p : free_brush_area_)
        {
            p.x += x;
            p.y += y;
            if (!setInteractiveGridValue(p.x, p.y, 255, 255, 255))
            {
                continue;
            }
            if (!setGridValueFromDisp(p.x, p.y, 255))
            {
                continue;
            }
            _normalize_xy(p.x, p.y, p.x, p.y);
            obstacles_.erase(Obstacle (p.x, p.y, p.cost));
        }
    }

    void Environment::markObstacle(int x, int y, int obstacle_radius)
    {
        for (auto p : cost_area_)
        {
            p.x += x;
            p.y += y;
            _normalize_xy(p.x, p.y, p.x, p.y);
            auto val = getCost(p.x, p.y);
            if (val <= p.cost)
            {
                continue;
            }
            if (!setInteractiveGridValue(p.x, p.y, p.cost, p.cost, p.cost))
            {
                continue;
            }
            if (!setGridValueFromDisp(p.x, p.y, p.cost))
            {
                continue;
            }

            auto tobst = Obstacle (p.x, p.y, p.cost);
            if ( obstacles_.find(tobst) != obstacles_.end())
            {
                obstacles_.erase(tobst);
            }
            obstacles_.insert(tobst);
        }
    }

    void Environment::showStartGoalPose()
    {
//        if (have_start_)
        {
            setInteractiveGridValue(start_x_, start_y_, start_r_, start_g_, start_b_);
        }
//        if (have_goal_)
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

    bool Environment::setIntGridValByPlanXY(int x, int y, uint8_t r, uint8_t g, uint8_t b, uint8_t a)
    {
        auto ix = x * rect_size_;
        auto iy = y * rect_size_;

        return setInteractiveGridValue(ix, iy, r, g, b, a);
    }

    void Environment::clear()
    {
        {
            std::lock_guard<std::mutex> lg(display_img_mtx_);
            display_img_ = cv::Mat(img_length_, img_width_, CV_8UC4, cv::Scalar(255, 255, 255, 255));
        }
        showStartGoalPose();
        for (auto &o : obstacles_)
        {
            setInteractiveGridValue(std::get<0>(o), std::get<1>(o), std::get<2>(o), std::get<2>(o), std::get<2>(o));
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
        display_img_ = cv::Mat(img_length_, img_width_, CV_8UC4, cv::Scalar(255, 255, 255, 255));
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
        Path tpath;
        adjustPlanResolution(path, tpath, 0.5);
        std::cout << path.size() << " " << tpath.size() <<std::endl;
        int x, y;
        cv::Mat display_copy;
        display_img_mtx_.lock();
        display_img_.copyTo(display_copy);
        display_img_mtx_.unlock();
        cv::Mat covered_mark = cv::Mat(img_length_, img_width_, CV_8UC4, cv::Scalar(255, 255, 255, 255));
        cv::Mat robot_pos = cv::Mat(img_length_, img_width_, CV_8UC4, cv::Scalar(255, 255, 255, 255));
        cv::Mat temp_img = cv::Mat(img_length_, img_width_, CV_8UC4, cv::Scalar(255, 255, 255, 255));
        for (const auto& n : tpath)
        {
            x = n.x * rect_size_;
            y = n.y * rect_size_;
//            setInteractiveGridValue(nx, ny, n.r, n.g, n.b, 50);

            _normalize_xy(x, y, x, y);

            // 标记已覆盖区域
            circle(covered_mark, cv::Point(x, y), robot_radius_ * rect_size_,
                   cv::Scalar(n.b, n.g, n.r, n.a), -1);
            // 标记已当前航点
            rectangle(display_copy, cv::Rect(x, y, rect_size_, rect_size_),
                      cv::Scalar(n.b, n.g, n.r, n.a), -1);
            // 绘制机器人
            circle(robot_pos, cv::Point(x, y), robot_radius_ * rect_size_,
                   cv::Scalar(0, 0, 255, 0), 3);
            // 融合交互图像与已覆盖区域图像
            cv::addWeighted(display_copy, 0.8, covered_mark, 0.2, 0.0, temp_img, -1);
            // 继续融合机器人图像
            display_img_mtx_.lock();
            cv::addWeighted(temp_img, 0.8, robot_pos, 0.2, 0.0, display_img_,  -1);
            display_img_mtx_.unlock();

            // 清除上一次的机器人位置图像
            circle(robot_pos, cv::Point(x, y), robot_radius_ * rect_size_,
                   cv::Scalar(255, 255, 255, 255), 3);

            std::this_thread::sleep_for(std::chrono::microseconds((int)(getDisplayDelayTime() * 1e6)));
            if (!is_running_.load())
            {
                break;
            }
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

    bool Environment::saveEnvironmntToDisk(std::string path)
    {
        std::cout << "Saving environment..." << std::endl;
        //根节点
        Json::Value root, obstacle;

        root["EnvironmentVersion"] = ENVIRONMENT_VERSION;

        //根节点属性
        root["start_x"] = start_x_;
        root["start_y"] = start_y_;
        root["goal_x"] = goal_x_;
        root["goal_y"] = goal_y_;
        root["size_x"] = getGridXSizeInCells();
        root["size_y"] = getGridYSizeInCells();
        root["display_scale"] = getScale();
        root["algorithm_index"] = getCurrentAlgorithmIndex();
        root["display_delay_time"] = getDisplayDelayTime();

        for (const auto &o : obstacles_)
        {
            obstacle[0] = std::get<0>(o);
            obstacle[1] = std::get<1>(o);
            obstacle[2] = std::get<2>(o);
            root["obstacles"].append(obstacle);
        }

        Json::StyledWriter sw;

        //输出到文件
        std::ofstream os;
        os.open(path, std::ios::out | std::ios::trunc);
        if (!os.is_open())
        {
            std::cerr << "Error：can not find or create the file which named \" environment.json\"." << std::endl;
            return false;
        }
        os << sw.write(root);
        os.close();
        std::cout << "Save environment finished." << std::endl;
        return true;
    }

    bool Environment::loadEnvironmentFromDisk(std::string path)
    {
        std::cout << "Loading environment..." << std::endl;
        Json::Reader reader;
        Json::Value root;

        //从文件中读取，保证当前文件有demo.json文件
        std::ifstream in(path, std::ios::in);

        if (!in.is_open())
        {
            std::cout << "Error opening file\n";
            return false;
        }

        if (!reader.parse(in, root))
        {
            std::cerr << "Environment config file parse failed." << std::endl;
            return false;
        }

        start_x_ =  root["start_x"].asInt();
        start_y_ =  root["start_y"].asInt();
        goal_x_ =  root["goal_x"].asInt();
        goal_y_ =  root["goal_y"].asInt();
        length_ = root["size_y"].asInt();
        width_ = root["size_x"].asInt();
        rect_size_ = root["display_scale"].asInt();
        algorithm_index_ = root["algorithm_index"].asInt();
        display_delay_time_ = root["display_delay_time"].asFloat();

        std::cout << "window length " << length_ << " window width " << width_ << " display scale " << rect_size_ << std::endl;
        std::cout << "Start x " << start_x_ << std::endl;
        std::cout << "Start y " << start_y_ << std::endl;
        std::cout << "Goal x " << goal_x_ << std::endl;
        std::cout << "Goal y " << goal_y_ << std::endl;
        std::cout << "Obstacle grids size " << root["obstacles"].size() << std::endl;
        std::cout << "Current algorithm index " << algorithm_index_ << std::endl;

        initialize(length_, width_, rect_size_);
        obstacles_.clear();
        for (const auto &o: root["obstacles"])
        {
            auto x = o[0].asInt();
            auto y = o[1].asInt();
            unsigned char cost = o[2].asUInt();
            obstacles_.insert(Obstacle(x, y, cost));
            if (!setGridValueFromDisp(x, y, cost))
            {
                continue;
            }
            setInteractiveGridValue(x, y, cost);
        }

        in.close();
        clear();
        showStartGoalPose();
        std::cout << "Load finished." << std::endl;
        return true;
    }

    cv::Mat *Environment::_get_planning_container(int x, int y)
    {
        if (x >= 0 && y >= 0)
        {
            return &pi_pp_;
        }
        else if (x >= 0 && y < 0)
        {
            return &pi_pn_;
        }
        else if (x < 0 && y >= 0)
        {
            return &pi_np_;
        }
        else if (x < 0 && y < 0)
        {
            return &pi_nn_;
        }
        else
        {
            std::cout << "Should not be here." << std::endl;
        }

        return nullptr;
    }

    cv::Mat *Environment::_get_display_container(int x, int y)
    {
        if (x >= 0 && y >= 0)
        {
            return &di_pp_;
        }
        else if (x >= 0 && y < 0)
        {
            return &di_pn_;
        }
        else if (x < 0 && y >= 0)
        {
            return &di_np_;
        }
        else if (x < 0 && y < 0)
        {
            return &di_nn_;
        }
        else
        {
            std::cout << "Should not be here." << std::endl;
        }

        return nullptr;
    }

    bool Environment::generateFootprintByRadius()
    {
        if (robot_radius_ <= 0.0f)
        {
            std::cerr << "Robot radius too small, can't generate footprint." << std::endl;
            return false;
        }
        GridPoint p{0};
        footprint_.clear();
        std::cout << "Robot radius " << robot_radius_ <<std::endl;
        for (int i = -robot_radius_+1; i < robot_radius_; i++)
        {
            for (int j = -robot_radius_+1; j < robot_radius_; j++)
            {
                if ((i*i + j*j) <= robot_radius_*robot_radius_)
                {
                    p.x = i;
                    p.y = j;
                    footprint_.emplace_back(p);
                }
            }
        }
        std::cout << "Robot footprint points size " << footprint_.size() << std::endl;
        return true;
    }

    const Footprint &Environment::getFootprint()
    {
        return footprint_;
    }

    void Environment::setRobotRadius(int robot_radius)
    {
        robot_radius_ = robot_radius;
        GridPoint p{0};
        cost_area_.clear();
        for (int i = -cost_radius_; i <= cost_radius_; i++)
        {
            for (int j = -cost_radius_; j <= cost_radius_; j++)
            {
                auto square = (i*i + j*j);
                if (square > cost_radius_*cost_radius_)
                {
                    continue;
                }
                p.x = i * rect_size_;
                p.y = j * rect_size_;
                _normalize_xy(p.x, p.y, p.x, p.y);
                if (square <= robot_radius_*robot_radius_)
                {
                    if (p.x == 0 && p.y == 0)
                    {
                        p.cost = LETHAL_OBSTACLE;
                    }
                    else
                    {
                        p.cost = INSCRIBED_INFLATED_OBSTACLE;
                    }
                    cost_area_.emplace_back(p);
                }
                else
                {
                    p.cost = 255 - exp(-1.0 * cost_scaling_factor * (sqrt(square) - robot_radius_)) * (PENALTY_COST);
                    cost_area_.emplace_back(p);
                }
            }
        }
    }

    void Environment::toGrid(int x, int y, int &outx, int &outy)
    {
        outx = x / rect_size_;
        outy = y / rect_size_;
    }

    void Environment::fromGrid(int x, int y, int &outx, int &outy)
    {
        outx = x * rect_size_;
        outy = y * rect_size_;
    }
}
