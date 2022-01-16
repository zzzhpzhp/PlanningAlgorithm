#pragma once
#include <cmath>
#include <iostream>
#include <memory>
#include <tuple>
#include <unordered_map>
#include <queue>
#include <assert.h>

namespace environment
{
    /**
     * @brief 路径航点定义
     * 
     */
    struct PathNode
    {
        // 航点坐标（规划界面坐标）
        int x, y;
        // 航点的颜色
        uint8_t r, g, b, a;
    };

    /**
     * @brief 路径定义
     * 
     */
    using Path = std::vector<PathNode>;

    /**
     * @brief 物理点定义
     * 
     */
    struct Point
    {
        // 点坐标（物理坐标）
        float x, y;
    };

    /**
     * @brief 栅格点定义（交互界面点）
     * 
     */
    struct GridPoint
    {
        // 点坐标（交互界面坐标）
        int x, y;
        // 点的值
        unsigned char cost;
    };

    /**
     * @brief 物理位姿定义
     * 
     */
    struct Pose
    {
        // 位姿坐标（物理坐标）
        float x, y;
        // 位姿航向（单位为弧度）
        float theta;
    };

    /**
     * @brief 物理多边形定义
     * 
     */
    using Polygon = std::vector<Point>;

    /**
     * @brief 网格（交互界面）多边形
     * 
     */
    using GridPolygon = std::vector<GridPoint>;
    
    /**
     * @brief 网格（交互界面）点集定义
     * 
     */
    using Grids = std::vector<GridPoint>;

    /**
     * @brief 机器人轮廓定义
     * 
     */
    using Footprint = std::vector<GridPoint>;

    /**
     * @brief 多边形边定义
     * 
     */
    struct Edge
    {
        // 最大y坐标
        float ymax{};
        // x坐标
        float x{};
        // 1/k的值
        float inv_k{};
        // 与之相连的下一条边
        std::shared_ptr<Edge> next;

        bool operator()(const std::shared_ptr<Edge> &a, const std::shared_ptr<Edge> &b) const
        {
            return a->x > b->x;
        };
    };

    struct Quaternion
    {
        float x{0.0f}, y{0.0f}, z{0.0f}, w{1.0f};

        Quaternion(){};
        Quaternion(float tx, float ty, float tz, float tw)
        {
            x = tx;
            y = ty;
            z = tz;
            w = tw;
        }

        /**@brief Set the quaternion using fixed axis RPY
         * @param roll Angle around X
         * @param pitch Angle around Y
         * @param yaw Angle around Z*/
        void setRPY(const float& roll, const float& pitch, const float& yaw)
        {
            float halfYaw = yaw * 0.5f;
            float halfPitch = pitch * 0.5f;
            float halfRoll = roll * 0.5f;
            float cosYaw = std::cos(halfYaw);
            float sinYaw = std::sin(halfYaw);
            float cosPitch = std::cos(halfPitch);
            float sinPitch = std::sin(halfPitch);
            float cosRoll = std::cos(halfRoll);
            float sinRoll = std::sin(halfRoll);

            x = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw; //x
            y = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw; //y
            z = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw; //z
            w = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw; //formerly yzx
        }

        void
        normalize()
        {
            if (isNornalized())
            {
                return;
            }

            float length = std::sqrt(x * x + y * y + z * z + w * w);
            if (length == 0.0)
            {
                std::cerr << "Quaternion length is zero." << std::endl;
                return;
            }
            auto inv_length = 1.0f / length;
            x = x * inv_length;
            y = y * inv_length;
            z = z * inv_length;
            w = w * inv_length;
        }

        void
        getRPY(float &roll, float &pitch, float &yaw) const
        {
            roll = std::atan2(2 * (w * z + x * y), 1 - 2 * (z * z + x * x));
            pitch = std::asin(2 * (w * x - y * z));
            yaw = std::atan2(2 * (w * y + z * x), 1 - 2 * (x * x + y * y));
        }

        bool
        isNornalized() const
        {
            return (std::sqrt(x * x + y * y + z * z + w * w) == 1.0f);
        }
    };

    struct Displacement
    {
        float x, y, z;
    };


    struct Transform
    {
        Displacement position;
        Quaternion rotation;
    };

    struct Vector3
    {
        float m_floats[3]{};
        Vector3(){}
        Vector3(float a, float b, float c)
        {
            m_floats[0] = a;
            m_floats[1] = b;
            m_floats[2] = c;
        }

        void
        setValue(float a, float b, float c)
        {
            m_floats[0] = a;
            m_floats[1] = b;
            m_floats[2] = c;
        }

        float&
        operator[](int i)
        {
            assert(0 <= i && i < 3);
            return m_floats[i];
        }

    };

    struct Matrix3x3
    {
        Vector3 m_vectors[3];

        Vector3&
        operator[](int i)
        {
            assert(0 <= i && i < 3);
            return m_vectors[i];
        }

        void
        setValue(Vector3 a, Vector3 b, Vector3 c)
        {
            m_vectors[0] = a;
            m_vectors[1] = b;
            m_vectors[2] = c;
        }

        void
        setValue(float xx, float xy, float xz,
                 float yx, float yy, float yz,
                 float zx, float zy, float zz)
        {
            m_vectors[0][0] = xx;
            m_vectors[0][1] = xy;
            m_vectors[0][2] = xz;
            m_vectors[1][0] = yx;
            m_vectors[1][1] = yy;
            m_vectors[1][2] = yz;
            m_vectors[2][0] = zx;
            m_vectors[2][1] = zy;
            m_vectors[2][2] = zz;
        }

        Vector3
        getColumn(int i)
        {
            assert(0 <= i && i < 3);
            return Vector3{m_vectors[0][i],m_vectors[1][i],m_vectors[2][i]};;
        }

        Vector3
        getRow(int i)
        {
            assert(0 <= i && i < 3);
            return m_vectors[i];
        }
//
//        void
//        setRotation(const Quaternion& q)
//        {
//            float d = q.length2();
//            assert(d != float(0.0));
//            float s = float(2.0) / d;
//            float xs = q.x * s,   ys = q.y * s,   zs = q.z * s;
//            float wx = q.w * xs,  wy = q.w * ys,  wz = q.w * zs;
//            float xx = q.x * xs,  xy = q.x * ys,  xz = q.x * zs;
//            float yy = q.y * ys,  yz = q.y * zs,  zz = q.z * zs;
//            setValue(float(1.0) - (yy + zz), xy - wz, xz + wy,
//                     xy + wz, float(1.0) - (xx + zz), yz - wx,
//                     xz - wy, yz + wx, float(1.0) - (xx + yy));
//        }
//        Vector3
//        operator*()
    };
}