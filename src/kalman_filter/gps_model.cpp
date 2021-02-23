#include "gps_model.h"

#include <ugl/math/matrix.h>
#include <ugl/lie_group/rotation.h>
#include <ugl/lie_group/extended_pose.h>

namespace invariant
{

const ugl::Matrix<3,3> GpsModel::s_noise_jacobian = ugl::Matrix<3,3>::Identity();

GpsModel::GpsModel(const MeasurementType& offset, const ugl::Matrix<3,3>& noise_covariance)
    : m_target(offset)
    , m_noise_covariance(noise_covariance)
    , m_modified_noise_covariance(s_noise_jacobian * noise_covariance * s_noise_jacobian.transpose())
{
    m_error_jacobian = ugl::Matrix<3,9>::Zero();
    m_error_jacobian.block<3,3>(0,0) = -ugl::lie::skew(m_target.vector());
    m_error_jacobian.block<3,3>(0,6) = ugl::Matrix3::Identity();
}

ugl::Vector3 GpsModel::h(const ugl::lie::ExtendedPose& state, const ugl::Vector3& noise) const
{
    return (group_action(state, m_target) * MeasurementType::exp(noise)).vector();
}

GpsModel::MeasurementType GpsModel::group_action(const ugl::lie::ExtendedPose& actor, const GpsModel::MeasurementType& target)
{
    return GpsModel::MeasurementType{actor.position() + actor.rotation() * target.vector()};
}

const GpsModel::MeasurementType& GpsModel::target() const
{
    return m_target;
}

const ugl::Matrix<3,9>& GpsModel::error_jacobian() const
{
    return m_error_jacobian;
}

const ugl::Matrix<3,3>& GpsModel::noise_jacobian()
{
    return s_noise_jacobian;
}

const ugl::Matrix<3,3>& GpsModel::noise_covariance() const
{
    return m_noise_covariance;
}

const ugl::Matrix<3,3>& GpsModel::modified_noise_covariance() const
{
    return m_modified_noise_covariance;
}

} // namespace invariant
