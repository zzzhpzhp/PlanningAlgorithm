#ifndef AFF_TRANSFORMER_H
#define AFF_TRANSFORMER_H

#include <iostream>
#include <cmath>

#include "typedefine.h"

namespace environment
{
    class Transformer
    {
    public:

        void
        getTransform(std::string target_frame, std::string origin_frame, Point& point);

        void
        getTransform(std::string target_frame, std::string origin_frame, float& x, float& y, float& z,
                     float& qx, float& qy, float& qz, float& qw);

        void
        setTransform(std::string parent_frame, std::string child_frame, float x, float y, float z, float theta);

        void
        setTransform(std::string parent_frame, std::string child_frame, float x, float y, float z,
                     float qx, float qy, float qz, float qw);

        void
        getTransform();

    private:

        std::unordered_map<std::string, std::unordered_map<std::string, Transform>> transform_data_;
    };
}

#endif //AFF_TRANSFORMER_H
