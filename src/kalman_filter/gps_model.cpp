#include "gps_model.h"

#include <ugl/math/matrix.h>
#include <ugl/lie_group/extended_pose.h>

namespace invariant
{

const GpsModel::MeasurementType GpsModel::s_target = []() {
    return GpsModel::MeasurementType::Identity();
}();

const ugl::Matrix<3,9> GpsModel::s_error_jacobian = []() {
    ugl::Matrix<3,9> jac = ugl::Matrix<3,9>::Zero();
    jac.block<3,3>(0,6) = ugl::Matrix3::Identity();
    return jac;
}();

const ugl::Matrix<3,3> GpsModel::s_noise_jacobian = []() {
    return ugl::Matrix<3,3>::Identity();
}();

const ugl::Matrix<3,3> GpsModel::s_noise_covariance = []() {
    constexpr double stddev = 0.1;  // [m]
    constexpr double variance = stddev*stddev;
    return ugl::Matrix<3,3>::Identity() * variance;
}();

ugl::Vector3 GpsModel::h(const ugl::lie::ExtendedPose& state)
{
    return group_action(state, target()).vector();
}

GpsModel::MeasurementType GpsModel::group_action(const ugl::lie::ExtendedPose& actor, const GpsModel::MeasurementType& target)
{
    return GpsModel::MeasurementType{actor.position() + actor.rotation() * target.vector()};
}

const GpsModel::MeasurementType& GpsModel::target()
{
    return s_target;
}

const ugl::Matrix<3,9>& GpsModel::error_jacobian()
{
    return s_error_jacobian;
}

const ugl::Matrix<3,3>& GpsModel::noise_jacobian()
{
    return s_noise_jacobian;
}

const ugl::Matrix<3,3>& GpsModel::noise_covariance()
{
    return s_noise_covariance;
}

} // namespace invariant
