#include "mocap_model.h"

#include <ugl/math/matrix.h>
#include <ugl/lie_group/pose.h>
#include <ugl/lie_group/extended_pose.h>

namespace invariant
{

const ugl::lie::Pose MocapModel::s_target = ugl::lie::Pose::Identity();

const ugl::Matrix<6,9> MocapModel::s_error_jacobian = []() {
    ugl::Matrix<6,9> jac = ugl::Matrix<6,9>::Zero();
    jac.block<3,3>(0,0) = ugl::Matrix3::Identity();
    jac.block<3,3>(3,6) = ugl::Matrix3::Identity();
    return jac;
}();

const ugl::Matrix<6,6> MocapModel::s_noise_jacobian = []() {
    return ugl::lie::Pose::adjoint(MocapModel::target());
}();

const ugl::Matrix<6,6> MocapModel::s_noise_covariance = []() {
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

const ugl::Matrix<6,9>& MocapModel::error_jacobian()
{
    return s_error_jacobian;
}

const ugl::Matrix<6,6>& MocapModel::noise_jacobian()
{
    return s_noise_jacobian;
}

const ugl::Matrix<6,6>& MocapModel::noise_covariance()
{
    return s_noise_covariance;
}

} // namespace invariant
