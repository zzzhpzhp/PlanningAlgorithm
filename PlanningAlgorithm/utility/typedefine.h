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
        unsigned char cost;
    };

    struct Pose
    {
        float x, y, theta;
    };

    using Polygon = std::vector<Point>;
    using GridPolygon = std::vector<GridPoint>;
    using Grids = std::vector<GridPoint>;
    using Footprint = std::vector<GridPoint>;


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