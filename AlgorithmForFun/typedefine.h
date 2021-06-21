#ifndef AFF_TYPEDEFINE_H
#define AFF_TYPEDEFINE_H

#include <cmath>
#include <iostream>
#include <memory>
#include <tuple>
#include <unordered_map>
#include <queue>

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
}
#endif //AFF_TYPEDEFINE_H
