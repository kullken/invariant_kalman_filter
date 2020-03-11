#pragma once

#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "iekf_types.h"

namespace tf2
{

template<>
inline
void fromMsg(const geometry_msgs::Quaternion& in, invariant::Rotation& out)
{
    invariant::Quaternion temp_q;
    tf2::fromMsg(in, temp_q);
    out = temp_q.toRotationMatrix();
}

inline
invariant::Rotation fromMsg(const geometry_msgs::Quaternion& in)
{
    invariant::Rotation out;
    tf2::fromMsg(in, out);
    return out;
}

inline
invariant::Vector3 fromMsg(const geometry_msgs::Point& in)
{
    invariant::Vector3 out;
    tf2::fromMsg(in, out);
    return out;
}

inline
invariant::Vector3 fromMsg(const geometry_msgs::Vector3& in)
{
    invariant::Vector3 out;
    tf2::fromMsg(in, out);
    return out;
}

// template<>
// geometry_msgs::Quaternion toMsg(const invariant::Rotation& R)
// {
//     geometry_msgs::Quaternion q = tf2::toMsg(invariant::Quaternion(R));
//     return q;
// }

template<>
geometry_msgs::Vector3 toMsg(const invariant::Vector3& in)
{
    geometry_msgs::Vector3 out;
    out.x = in.x();
    out.y = in.y();
    out.z = in.z();
    return out;
}

inline
geometry_msgs::Quaternion& toMsg(const invariant::Rotation& in, geometry_msgs::Quaternion& out)
{
    out = tf2::toMsg(invariant::Quaternion(in));
    return out;
}

// inline
// geometry_msgs::Vector3& toMsg(const invariant::Vector3& in, geometry_msgs::Vector3& out)
// {
//   out.x = in.x();
//   out.y = in.y();
//   out.z = in.z();
//   return out;
// }

}