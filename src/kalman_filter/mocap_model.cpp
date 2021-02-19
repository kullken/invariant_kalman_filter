#include "mocap_model.h"

#include <ugl/math/matrix.h>
#include <ugl/lie_group/pose.h>
#include <ugl/lie_group/extended_pose.h>

namespace invariant
{

const ugl::Matrix<6,9> MocapModel::s_error_jacobian = []() {
    ugl::Matrix<6,9> jac = ugl::Matrix<6,9>::Zero();
    jac.block<3,3>(0,0) = ugl::Matrix3::Identity();
    jac.block<3,3>(3,6) = ugl::Matrix3::Identity();
    return jac;
}();

MocapModel::MocapModel(const ugl::lie::Pose& offset, const ugl::Matrix<6,6>& noise_covariance)
    : m_target(offset)
    , m_noise_covariance(noise_covariance)
    , m_modified_noise_covariance(noise_jacobian() * noise_covariance * noise_jacobian().transpose())
{
}

ugl::lie::Pose MocapModel::h(const ugl::lie::ExtendedPose& state, const ugl::lie::Pose::VectorType& noise) const
{
    return (group_action(state, m_target) * ugl::lie::Pose::exp(noise));
}

ugl::lie::Pose MocapModel::group_action(const ugl::lie::ExtendedPose& actor, const ugl::lie::Pose& target)
{
    return ugl::lie::Pose{actor.rotation(), actor.position()} * target;
}

const ugl::lie::Pose& MocapModel::target() const
{
    return m_target;
}

const ugl::Matrix<6,9>& MocapModel::error_jacobian()
{
    return s_error_jacobian;
}

ugl::Matrix<6,6> MocapModel::noise_jacobian() const
{
    return ugl::lie::Pose::adjoint(m_target); // TODO: Precompute in constructor?
}

const ugl::Matrix<6,6>& MocapModel::noise_covariance() const
{
    return m_noise_covariance;
}

const ugl::Matrix<6,6>& MocapModel::modified_noise_covariance() const
{
    return m_modified_noise_covariance;
}

} // namespace invariant
