#include "transformer.h"

namespace environment
{

    void
    Transformer::setTransform(std::string parent_frame, std::string child_frame, float x, float y, float z, float qx,
                              float qy, float qz, float qw)
    {
        transform_data_[parent_frame][child_frame].position.x = x;
        transform_data_[parent_frame][child_frame].position.y = y;
        transform_data_[parent_frame][child_frame].position.z = z;
        transform_data_[parent_frame][child_frame].rotation.x = qx;
        transform_data_[parent_frame][child_frame].rotation.y = qy;
        transform_data_[parent_frame][child_frame].rotation.z = qz;
        transform_data_[parent_frame][child_frame].rotation.w = qw;
    }

    void
    Transformer::setTransform(std::string parent_frame, std::string child_frame, float x, float y, float z, float theta)
    {
        transform_data_[parent_frame][child_frame].position.x = x;
        transform_data_[parent_frame][child_frame].position.y = y;
        transform_data_[parent_frame][child_frame].position.z = z;
        transform_data_[parent_frame][child_frame].rotation.setRPY(0.0f, 0.0f, theta);
    }

    void Transformer::getTransform(std::string target_frame, std::string origin_frame, float &x, float &y, float &z,
                                   float &qx, float &qy, float &qz, float &qw)
    {

    }

    void Transformer::getTransform(std::string target_frame, std::string origin_frame, Pose &pose)
    {

    }
}