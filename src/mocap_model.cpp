#include "mocap_model.h"

#include <ugl/math/matrix.h>
#include <ugl/lie_group/pose.h>
#include <ugl/lie_group/extended_pose.h>

namespace invariant
{

const ugl::lie::Pose MocapModel::s_target = ugl::lie::Pose::Identity();

const ugl::Matrix<6,9> MocapModel::s_H = []() {
    ugl::Matrix<6,9> H = ugl::Matrix<6,9>::Zero();
    H.block<3,3>(0,0) = ugl::Matrix3::Identity();
    H.block<3,3>(3,6) = ugl::Matrix3::Identity();
    return H;
}();

const ugl::Matrix<6,6> MocapModel::s_G = []() {
    return ugl::lie::Pose::adjoint(MocapModel::target());
}();

const ugl::Matrix<6,6> MocapModel::s_N = []() {
    return ugl::Matrix<6,6>::Identity() * 0.01;
}();

ugl::lie::Pose MocapModel::group_action(const ugl::lie::ExtendedPose& actor, const ugl::lie::Pose& target)
{
    return ugl::lie::Pose{actor.rotation(), actor.position()} * target;
}

const ugl::lie::Pose& MocapModel::target()
{
    return s_target;
}

const ugl::Matrix<6,9>& MocapModel::H()
{
    return s_H;
}

const ugl::Matrix<6,6>& MocapModel::G()
{
    return s_G;
}

const ugl::Matrix<6,6>& MocapModel::N()
{
    return s_N;
}

} // namespace invariant
