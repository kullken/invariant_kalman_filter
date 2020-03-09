#pragma once

#include <tf2/convert.h>
// #include <tf2_eigen/tf2_eigen.h>

#include <geometry_msgs/Quaternion.h>

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

// template<>
// geometry_msgs::Quaternion toMsg(const invariant::Rotation& R)
// {
//     geometry_msgs::Quaternion q = tf2::toMsg(invariant::Quaternion(R));
//     return q;
// }

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
