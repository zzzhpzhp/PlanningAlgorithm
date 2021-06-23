#include "transformer.h"

namespace environment
{

    void
    Transformer::setTransform(std::string parent_frame, std::string child_frame, float x, float y, float z, float qx,
                              float qy, float qz, float qw)
    {
        auto &trans = transform_data_[parent_frame][child_frame];
        trans.position.x = x;
        trans.position.y = y;
        trans.position.z = z;
        trans.rotation.x = qx;
        trans.rotation.y = qy;
        trans.rotation.z = qz;
        trans.rotation.w = qw;
    }

    void
    Transformer::setTransform(std::string parent_frame, std::string child_frame, float x, float y, float z, float theta)
    {
        auto &trans = transform_data_[parent_frame][child_frame];
        trans.position.x = x;
        trans.position.y = y;
        trans.position.z = z;
        trans.rotation.setRPY(0.0f, 0.0f, theta);
    }

    void Transformer::getTransform(std::string target_frame, std::string origin_frame, float &x, float &y, float &z,
                                   float &qx, float &qy, float &qz, float &qw)
    {

    }

    void Transformer::getTransform(std::string target_frame, std::string origin_frame, Pose &pose)
    {

    }
}











