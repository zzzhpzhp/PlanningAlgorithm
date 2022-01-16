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

// 调试用宏定义
#define FL_PRINT {std::cout << __FUNCTION__ << " " << __LINE__ << std::endl;};

namespace environment
{
    /**
     * @brief 网格的值定义
     * 
     */
    const int NO_INFORMATION = 150;
    const int LETHAL_OBSTACLE = 0;
    const int INSCRIBED_INFLATED_OBSTACLE = 50;
    const int PENALTY_COST = 100;
    const int FREE_SPACE = 255;
    const int CLEANED_COST = 230;

    /**
     * @brief 标记动作类型
     * 
     */
    enum MarkMode {MARK_OBSTACLE, MARK_FREE_SPACE};

    /**
     * @brief 环境接口类，定义二维环境下的规划算法运行环境。
              环境由交互界面和规划界面组成，交互界面可通过鼠标键盘等工具标记起点、目标点和障碍物等，还可显示规划过程和规划结果；
              规划界面仅显示和规划相关的事物，如障碍物和障碍物腐蚀区域（代价区域）等。
              交互界面和规划界面存在一个比例缩放关系，既规划界面中一个像素，在交互界面中可能由多个像素组成，具体的比例由配置决定。
     * 
     */
    class EnvironmentInterface
    {
    public:
        /**
         * @brief 初始化环境
         * 
         * @param length 环境的长度
         * @param width  环境的宽度
         * @param display_scale 每单位尺寸的像素数量
         */
        virtual void
        initialize(int length, int width, int display_scale) = 0;

        /**
         * @brief 根据交互界面的坐标得到规划网格内对应坐标的网格的值
         * 
         * @param x 交互界面的x坐标
         * @param y 交互界面的y坐标
         * @return uint8_t 对应规划网格的值
         */
        virtual uint8_t
        getGridValueFromDisp(int x, int y) = 0;

        /**
         * @brief 通过交互界面的坐标设置对应的网格坐标的值
         * 
         * @param x 规划界面x坐标
         * @param y 规划界面y坐标
         * @param value 待设置的值
         * @return true 设置成功
         * @return false 设置失败
         */
        virtual bool
        setGridValueFromDisp(int x, int y, uint8_t value) = 0;

        /**
         * @brief 获取规划网格的值
         * 
         * @param x 网格x坐标
         * @param y 网格y坐标
         * @return uint8_t 网格的值
         */
        virtual uint8_t
        getGridValue(int x, int y) = 0;

        /**
         * @brief 设置规划界面的值
         * 
         * @param x 网格x坐标
         * @param y 网格y坐标
         * @param value 待设置的值
         * @return true 设置成功
         * @return false 设置失败
         */
        virtual bool
        setGridValue(int x, int y, uint8_t value) = 0;

        /**
         * @brief 将交互界面坐标转化为规划界面坐标
         * 
         * @param x 交互界面x坐标
         * @param y 交互界面y坐标
         * @param outx 规划界面x坐标
         * @param outy 规划界面y坐标
         */
        virtual void
        toGrid(int x, int y, int &outx, int &outy) = 0;

        /**
         * @brief 将交互界面坐标转化为规划界面坐标
         * 
         * @param raw_x 交互界面x坐标
         * @param raw_y 交互界面y坐标
         * @param x 规划界面x坐标
         * @param y 规划界面y坐标
         * @return true 转换得到的坐标在规划界面内
         * @return false 转换得到的坐标不在规划界面内
         */
        virtual bool
        toGridAndInsideGrid(int raw_x, int raw_y, int &x, int &y) = 0;

        /**
         * @brief 将规划界面坐标转化为交互界面坐标
         * 
         * @param x 规划界面x坐标
         * @param y 规划界面y坐标
         * @param outx 交互界面x坐标
         * @param outy 交互界面y坐标
         */
        virtual void
        fromGrid(int x, int y, int &outx, int &outy) = 0;


        /**
         * @brief 获取机器人半径
         * 
         * @return int 机器人半径
         */
        virtual int
        getRobotRadius()
        {
            return robot_radius_;
        }

        /**
         * @brief 获取每一网格单位尺寸表示的物理距离
         * 
         * @return float 物理距离
         */
        virtual float
        getResolution() = 0;

        /**
         * @brief 设置机器人半径
         * 
         * @param robot_radius 
         */
        virtual void
        setRobotRadius(int robot_radius) = 0;

        /**
         * @brief 设置算法执行每步骤延时时间
         * 
         * @param second 延时时间，单位：秒
         */
        virtual void
        setAlgorithmRunningDelayTime(float second)
        {
            delay_time_ = second;
        }

        /**
         * @brief 设置结果演示每步骤延时时间
         * 
         * @param second 延时时间，单位：秒
         */
        virtual void
        setDisplayDelayTime(float second)
        {
            display_delay_time_ = second;
        }

        /**
         * @brief 获取结果演示每步骤延时时间
         * 
         * @return float 延时时间，单位：秒
         */
        virtual float
        getDisplayDelayTime()
        {
            return display_delay_time_;
        }

        /**
         * @brief 获取算法执行每步骤延时时间
         * 
         * @return double 延时时间，单位：秒
         */
        virtual double
        getAlgorithmRunningDelayTime()
        {
            return delay_time_;
        }

        /**
         * @brief 将当前环境配置信息保存到磁盘文件中
         * 
         * @param path 文件路径
         * @return true 保存成功
         * @return false 保存失败
         */
        virtual bool
        saveEnvironmntToDisk(std::string path) = 0;

        /**
         * @brief 从磁盘文件中加载环境配置信息
         * 
         * @param path 文件路径
         * @return true 加载成功
         * @return false 加载失败
         */
        virtual bool
        loadEnvironmentFromDisk(std::string path) = 0;

        /**
         * @brief 通过交互界面坐标设置交互界面中的网格的颜色
         * 
         * @param x 交互界面x坐标
         * @param y 交互界面y坐标
         * @param r 网格红色分量值
         * @param g 网格绿色分量值
         * @param b 网格蓝色分量值
         * @param a 网格透明度
         * @return true 设置成功
         * @return false 设置失败
         */
        virtual bool
        setInteractiveGridValue(int x, int y, uint8_t r = 0, uint8_t g = 0, uint8_t b = 0, uint8_t a = 0) = 0;

        /**
         * @brief 通过规划界面坐标设置交互界面中的网格的值
         * 
         * @param x 规划界面x坐标
         * @param y 规划界面y坐标
         * @param r 网格红色分量值
         * @param g 网格绿色分量值
         * @param b 网格蓝色分量值
         * @param a 网格透明度
         * @return true 设置成功
         * @return false 设置失败
         */
        virtual bool
        setIntGridValueByGridXY(int x, int y, uint8_t r = 0, uint8_t g = 0, uint8_t b = 0, uint8_t a = 70) = 0;

        /**
         * @brief 在交互界面中绘制路径，路径中的点坐标由规划界面坐标组成。
         * 
         * @param path 路径
         */
        virtual void
        drawPath(const Path &path) = 0;

        /**
         * @brief 获取规划界面X方向的网格数量
         * 
         * @return int 网格数量
         */
        virtual int
        getGridXSizeInCells() = 0;

        /**
         * @brief 获取规划界面y方向的网格数量
         * 
         * @return int 网格数量
         */
        virtual int
        getGridYSizeInCells() = 0;

        /**
         * @brief 判断坐标是否在规划界面内
         * 
         * @param x x坐标
         * @param y y坐标
         * @return true 坐标在规划界面内
         * @return false 坐标不在规划界面内
         */
        virtual bool
        insideGrid(int x, int y) = 0;

        /**
         * @brief 判断交互界面的坐标转化为规划界面的坐标后是否在规划界面内
         *
         * @param x 交互界面x坐标
         * @param y 交互界面y坐标
         * @return true 在规划界面内
         * @return false 不在规划界面内
         */
        virtual bool
        insideGridFromDisp(int x, int y) = 0;

        /**
         * @brief 获取交互界面相对于规划界面的缩放比例
         *
         * @return int 缩放比例
         */
        virtual int
        getScale() = 0;

        /**
         * @brief 刷新交互界面和规划界面
         *
         */
        virtual void
        display() = 0;

        /**
         * @brief 清理环境
         * 
         */
        virtual void
        clear() = 0;

        /**
         * @brief 复位环境
         * 
         */
        virtual void
        reset() = 0;

        /**
         * @brief 启动环境运行（开启界面交互反应和界面刷新等）
         * 
         */
        virtual void
        start()
        {
            is_running_.store(true);
        }

        /**
         * @brief 停止环境运行
         * 
         */
        virtual void
        stop()
        {
            is_running_.store(false);
        }

        /**
         * @brief 查询环境是否正在运行
         * 
         * @return true 环境正在运行
         * @return false 环境停止
         */
        virtual bool
        isRunning()
        {
            return is_running_.load();
        }

        /**
         * @brief 标记起点
         * 
         * @param x 起点x坐标
         * @param y 起点y坐标
         * @param r 起点颜色红色分量
         * @param g 起点颜色绿色分量
         * @param b 起点颜色蓝色分量
         */
        virtual void
        markStart(int x, int y, int r = 0, int g = 0, int b = 0) = 0;

        /**
         * @brief 标记目标点
         * 
         * @param x 目标点x坐标
         * @param y 目标点y坐标
         * @param r 目标点颜色红色分量
         * @param g 目标点颜色绿色分量
         * @param b 目标点颜色蓝色分量
         */
        virtual void
        markGoal(int x, int y, int r = 0, int g = 0, int b = 0) = 0;

        /**
         * @brief 标记障碍物
         * 
         * @param x 障碍物中心x坐标
         * @param y 障碍物中心y坐标
         * @param obstacle_radius 障碍物半径
         */
        virtual void
        markObstacle(int x, int y, int obstacle_radius = 1) = 0;

        /**
         * @brief 标记可行区域
         * 
         * @param x 可行区域中心x坐标
         * @param y 可行区域中心y坐标
         * @param radius 可行区域半径
         */
        virtual void
        markFreeSpace(int x, int y, int radius = 1) = 0;

        /**
         * @brief 在交互界面显示起点和目标点
         * 
         */
        virtual void
        showStartGoalPose() = 0;

        /**
         * @brief 获取起点坐标
         * 
         * @return std::tuple<int, int> 起点x、y坐标
         */
        virtual std::tuple<int, int>
        getStart() = 0;

        /**
         * @brief 获取目标点坐标
         * 
         * @return std::tuple<int, int> 目标点x、y坐标
         */
        virtual std::tuple<int, int>
        getGoal() = 0;

        /**
         * @brief 演示规划结果
         * 
         * @param path 规划结果，路径内的点的坐标应为规划界面坐标
         */
        virtual void
        play(Path &path) = 0;

        /**
         * @brief 设置机器人轮廓
         * 
         * @param footprint 轮廓
         */
        virtual void
        setFootprint(Footprint footprint) = 0;

        /**
         * @brief 设置坐标处机器人轮廓覆盖范围内的点的值
         * 
         * @param x 机器人轮廓内的点的坐标的x偏移量
         * @param y 机器人轮廓内的点的坐标的y偏移量
         * @param cost 待设置的值
         */
        virtual void
        setFootprintCost(int x, int y, unsigned char cost) = 0;

        /**
         * @brief 获取机器人的轮廓
         * 
         * @return const Footprint& 轮廓
         */
        virtual const Footprint&
        getFootprint() = 0;

        /**
         * @brief 根据机器人半径生成机器人轮廓
         * 
         * @return true 生成成功
         * @return false 生成失败
         */
        virtual bool
        generateFootprintByRadius() = 0;

        /**
         * @brief 在交互界面绘制多边形，点的坐标应为规划界面坐标
         * 
         * @param polygon 多边形
         */
        virtual void
        drawPolygon(const GridPolygon &polygon) = 0;

        /**
         * @brief 在交互界面和规划界面将多边形内的网格的填充为0值
         * 
         * @param polygon 多边形
         */
        virtual void
        fillGridPolygon(const GridPolygon& polygon) = 0;

        /**
         * @brief 在交互界面和规划界面绘制直线，坐标为网格坐标
         * 
         * @param x1 直线起点网格x坐标
         * @param y1 直线起点网格y坐标
         * @param x2 直线终点网格x坐标
         * @param y2 直线终点网格y坐标
         */
        virtual void
        drawGridLine(int x1, int y1, int x2, int y2) = 0;

        /**
         * @brief 
         * 
         * @param x1 
         * @param y1 
         * @param x2 
         * @param y2 
         */
        virtual void
        drawLine(int x1, int y1, int x2, int y2) = 0;

        /**
         * @brief 
         * 
         * @param polygon 
         */
        virtual void
        fillPolygonOutline(const GridPolygon& polygon) = 0;

        /**
         * @brief Get the Polygon Points object
         * 
         * @param polygon 
         * @return Grids 
         */
        virtual Grids
        getPolygonPoints(const GridPolygon &polygon) = 0;

        /**
         * @brief Get the Polygon Outline Points object
         * 
         * @param polygon 
         * @return Grids 
         */
        virtual Grids
        getPolygonOutlinePoints(const GridPolygon &polygon) = 0;

        /**
         * @brief 
         * 
         * @param points 
         */
        virtual void
        fillPoints(const Grids& points) = 0;

        /**
         * @brief 切换标记模式，在标记障碍物和标记可行区域之间切换
         * 
         */
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

        /**
         * @brief 获取标记模式
         * 
         * @return int 标记模式
         */
        virtual int
        getMarkMode()
        {
            return mark_mode_;
        };

        /**
         * @brief 获取当前运行算法的索引
         * 
         * @return int 
         */
        virtual int
        getCurrentAlgorithmIndex()
        {
            return algorithm_index_.load();
        }

        /**
         * @brief 设置当前算法的索引
         * 
         * @param index 索引
         */
        virtual void
        setCurrentAlgorithmIndex(int index)
        {
            algorithm_index_.store(index);
        }

        /**
         * @brief 将物理坐标转化为网格规划界面坐标
         * 
         * @param wx 物理x坐标
         * @param wy 物理y坐标
         * @param mx 规划界面x坐标
         * @param my 规划界面y坐标
         */
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

        /**
         * @brief 将规划界面坐标转化为物理坐标
         * 
         * @param mx 规划界面x坐标
         * @param my 规划界面y坐标
         * @param wx 物理x坐标
         * @param wy 物理y坐标
         */
        virtual void
        mapToWorld(int mx, int my, float &wx, float &wy)
        {
            wx = (float)mx * resolution_.load();
            wy = (float)my * resolution_.load();
        }

        /**
         * @brief 获取规划界面对应坐标处的值
         * 
         * @param x 规划界面x坐标
         * @param y 规划界面y坐标
         * @return unsigned char 值
         */
        virtual unsigned char
        getCost(int x, int y)
        {
            return getGridValueFromDisp(x, y);
        };

        /**
         * @brief 设置规划界面对应坐标处的值
         * 
         * @param x 规划界面x坐标
         * @param y 规划界面y坐标
         * @param cost 值
         */
        virtual void
        setCost(int x, int y, unsigned char cost) {};

        /**
         * @brief 获取规划界面对应坐标处的值
         * 
         * @param x 规划界面x坐标
         * @param y 规划界面y坐标
         * @return unsigned char 值
         */
        virtual unsigned char
        getCost(float x, float y)
        {
            int mx, my;
            worldToMap(x, y, mx, my);
            return getCost(mx, my);
        }

        /**
         * @brief 设置规划界面对应坐标处的值
         * 
         * @param x 规划界面x坐标
         * @param y 规划界面y坐标
         * @param cost 待设置的值
         */
        virtual void
        setCost(float x, float y, unsigned char cost)
        {
            int mx, my;
            worldToMap(x, y, mx, my);
            setCost(mx, my, cost);
        }

        /**
         * @brief 以给定的交互界面的坐标为中心绘制指定参数的圆
         * 
         * @param x 交互界面的圆心x坐标
         * @param y 交互界面的圆心y坐标
         * @param radius 圆的半径
         * @param line_width 绘制线条粗细
         * @param r 线条红色分量
         * @param g 线条绿色分量
         * @param b 线条蓝色分量
         * @param a 线条透明度
         */
        virtual void
        drawCircleFromDisp(int x, int y, int radius, int line_width = 1, int r = 0, int g = 0, int b = 0, int a = 255) = 0;

    protected:

        // 机器人半径
        std::atomic<int> robot_radius_{5};
        // 当前算法索引
        std::atomic<int> algorithm_index_{0};
        // 算法每步骤延时时间
        std::atomic<float> delay_time_{0.001};
        // 结果演示每步骤延时时间
        std::atomic<float> display_delay_time_{0.001};
        // 环境运行状态指示
        std::atomic_bool is_running_{false};
        // 标记状态
        std::atomic<int> mark_mode_{MARK_OBSTACLE};
        // 地图物理分辨率
        std::atomic<float> resolution_{0.05f};
    };

    using EnvironmentInterfacePtr = std::shared_ptr<EnvironmentInterface>;
}
