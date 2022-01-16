#pragma once
#include "environment_interface.h"

namespace algorithm
{
    /**
     * @brief 算法接口类。
     *        定义算法的基础API和通用的工具函数。
     * 
     */
    class AlgorithmInterface
    {
    public:

        /**
         * @brief 初始化算法
         * 
         * @param env 环境指针
         * @param name 算法名称
         */
        virtual void
        initialize(environment::EnvironmentInterfacePtr &env, std::string name) = 0;

        /**
         * @brief 获取算法名称
         * 
         * @return std::string 算法名称
         */
        virtual std::string
        getName()
        {
            return name_;
        }

        /**
         * @brief 设置目标坐标（规划界面坐标）
         * 
         * @param x 目标x坐标
         * @param y 目标y坐标
         */
        virtual void
        setGoal(int x, int y) {};

        /**
         * @brief 设置起点坐标（规划界面坐标）
         * 
         * @param x 起点x坐标
         * @param y 起点y坐标
         */
        virtual void
        setStart(int x, int y) {};

        /**
         * @brief 启动一次规划
         * 
         * @return true 规划成功
         * @return false 规划失败
         */
        virtual bool
        planning() {};

        /**
         * @brief 获取规划结果
         * 
         * @return environment::Path& 规划得到的路径
         */
        virtual environment::Path&
        getPath() {};

        /**
         * @brief 将算法设置为运行状态
         * 
         */
        virtual void
        start()
        {
            is_running_.store(true);
        }

        /**
         * @brief 将算法设置为停止状态
         * 
         */
        virtual void
        stop()
        {
            is_running_.store(false);
        }

    protected:

        /**
         * @brief 获取当前点的正上方的点的坐标（当前点坐标为规划界面坐标）
         * 
         * @param ep 环境指针
         * @param x 当前点x坐标
         * @param y 当前点y坐标
         * @param sx 对应方向的x坐标
         * @param sy 对应方向的y坐标
         * @param step_size 步长，表示目标点x方向、y方向距离当前点的距离
         * @return true 获取成功
         * @return false 获取失败
         */
        inline bool
        _get_middle_higher(environment::EnvironmentInterfacePtr &ep, int x, int y, int &sx, int &sy, int step_size = 1)
        {
            sx = x;
            sy = y - step_size;
            if (ep->insideGrid(sx, sy))
            {
                return true;
            }
            return false;
        }

        /**
         * @brief 获取当前点的正下方的点的坐标（当前点坐标为规划界面坐标）
         *
         * @param ep 环境指针
         * @param x 当前点x坐标
         * @param y 当前点y坐标
         * @param sx 对应方向的x坐标
         * @param sy 对应方向的y坐标
         * @param step_size 步长，表示目标点x方向、y方向距离当前点的距离
         * @return true 获取成功
         * @return false 获取失败
         */
        inline bool
        _get_middle_lower(environment::EnvironmentInterfacePtr &ep, int x, int y, int &sx, int &sy, int step_size = 1)
        {
            sx = x;
            sy = y + step_size;
            if (ep->insideGrid(sx, sy))
            {
                return true;
            }
            return false;
        }

        /**
         * @brief 获取当前点的左方的点的坐标（当前点坐标为规划界面坐标）
         *
         * @param ep 环境指针
         * @param x 当前点x坐标
         * @param y 当前点y坐标
         * @param sx 对应方向的x坐标
         * @param sy 对应方向的y坐标
         * @param step_size 步长，表示目标点x方向、y方向距离当前点的距离
         * @return true 获取成功
         * @return false 获取失败
         */
        inline bool
        _get_left(environment::EnvironmentInterfacePtr &ep, int x, int y, int &sx, int &sy, int step_size = 1)
        {
            sx = x - step_size;
            sy = y;
            if (ep->insideGrid(sx, sy))
            {
                return true;
            }
            return false;
        }

        /**
         * @brief 获取当前点的右方的点的坐标（当前点坐标为规划界面坐标）
         *
         * @param ep 环境指针
         * @param x 当前点x坐标
         * @param y 当前点y坐标
         * @param sx 对应方向的x坐标
         * @param sy 对应方向的y坐标
         * @param step_size 步长，表示目标点x方向、y方向距离当前点的距离
         * @return true 获取成功
         * @return false 获取失败
         */
        inline bool
        _get_right(environment::EnvironmentInterfacePtr &ep, int x, int y, int &sx, int &sy, int step_size = 1)
        {
            sx = x + step_size;
            sy = y;
            if (ep->insideGrid(sx, sy))
            {
                return true;
            }
            return false;
        }

        /**
         * @brief 获取当前点的左上方的点的坐标（当前点坐标为规划界面坐标）
         *
         * @param ep 环境指针
         * @param x 当前点x坐标
         * @param y 当前点y坐标
         * @param sx 对应方向的x坐标
         * @param sy 对应方向的y坐标
         * @param step_size 步长，表示目标点x方向、y方向距离当前点的距离
         * @return true 获取成功
         * @return false 获取失败
         */
        inline bool
        _get_higher_left(environment::EnvironmentInterfacePtr &ep, int x, int y, int &sx, int &sy, int step_size = 1)
        {
            sx = x - step_size;
            sy = y - step_size;
            if (ep->insideGrid(sx, sy))
            {
                return true;
            }
            return false;
        }

        /**
         * @brief 获取当前点的右上方的点的坐标（当前点坐标为规划界面坐标）
         *
         * @param ep 环境指针
         * @param x 当前点x坐标
         * @param y 当前点y坐标
         * @param sx 对应方向的x坐标
         * @param sy 对应方向的y坐标
         * @param step_size 步长，表示目标点x方向、y方向距离当前点的距离
         * @return true 获取成功
         * @return false 获取失败
         */
        inline bool
        _get_higher_right(environment::EnvironmentInterfacePtr &ep, int x, int y, int &sx, int &sy, int step_size = 1)
        {
            sx = x + step_size;
            sy = y - step_size;
            if (ep->insideGrid(sx, sy))
            {
                return true;
            }
            return false;
        }

        /**
         * @brief 获取当前点的左下方的点的坐标（当前点坐标为规划界面坐标）
         *
         * @param ep 环境指针
         * @param x 当前点x坐标
         * @param y 当前点y坐标
         * @param sx 对应方向的x坐标
         * @param sy 对应方向的y坐标
         * @param step_size 步长，表示目标点x方向、y方向距离当前点的距离
         * @return true 获取成功
         * @return false 获取失败
         */
        inline bool
        _get_lower_left(environment::EnvironmentInterfacePtr &ep, int x, int y, int &sx, int &sy, int step_size = 1)
        {
            sx = x - step_size;
            sy = y + step_size;
            if (ep->insideGrid(sx, sy))
            {
                return true;
            }
            return false;
        }

        /**
         * @brief 获取当前点的右下方的点的坐标（当前点坐标为规划界面坐标）
         *
         * @param ep 环境指针
         * @param x 当前点x坐标
         * @param y 当前点y坐标
         * @param sx 对应方向的x坐标
         * @param sy 对应方向的y坐标
         * @param step_size 步长，表示目标点x方向、y方向距离当前点的距离
         * @return true 获取成功
         * @return false 获取失败
         */
        inline bool
        _get_lower_right(environment::EnvironmentInterfacePtr &ep, int x, int y, int &sx, int &sy, int step_size = 1)
        {
            sx = x + step_size;
            sy = y + step_size;
            if (ep->insideGrid(sx, sy))
            {
                return true;
            }
            return false;
        }

    protected:
        // 算法名称
        std::string name_{};
        // 算法运行状态
        std::atomic_bool is_running_{true};
    };
}