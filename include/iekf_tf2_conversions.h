#pragma once

#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <ugl/math/vector.h>
#include <ugl/math/matrix.h>
#include <ugl/math/quaternion.h>

// TODO: Move to seperate "ugl_ros" package.

namespace tf2
{

template<>
inline
void fromMsg(const geometry_msgs::Quaternion& in, ugl::Rotation& out)
{
    ugl::Quaternion temp_q;
    tf2::fromMsg(in, temp_q);
    out = temp_q.toRotationMatrix();
}

inline
ugl::Rotation fromMsg(const geometry_msgs::Quaternion& in)
{
    ugl::Rotation out;
    tf2::fromMsg(in, out);
    return out;
}

inline
ugl::Vector3 fromMsg(const geometry_msgs::Point& in)
{
    ugl::Vector3 out;
    tf2::fromMsg(in, out);
    return out;
}

inline
ugl::Vector3 fromMsg(const geometry_msgs::Vector3& in)
{
    ugl::Vector3 out;
    tf2::fromMsg(in, out);
    return out;
}

template<>
inline
geometry_msgs::Vector3 toMsg(const ugl::Vector3& in)
{
    geometry_msgs::Vector3 out;
    out.x = in.x();
    out.y = in.y();
    out.z = in.z();
    return out;
}

inline
geometry_msgs::Quaternion& toMsg(const ugl::Rotation& in, geometry_msgs::Quaternion& out)
{
    out = tf2::toMsg(ugl::Quaternion(in));
    return out;
}

}
