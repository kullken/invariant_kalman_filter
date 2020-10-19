#include "offset_generator.h"

#include <ugl/math/vector.h>
#include <ugl/math/matrix.h>
#include <ugl/lie_group/extended_pose.h>

namespace invariant::test
{

const ugl::Matrix<9,9> OffsetGenerator::s_default_covariance = []() {
    constexpr double rot_stddev = 0.1;  // [rad]
    constexpr double vel_stddev = 0.5;  // [m/s]
    constexpr double pos_stddev = 0.5;  // [m]

    ugl::Matrix<9,9> covariance = ugl::Matrix<9,9>::Zero();
    covariance.block<3,3>(0,0) = ugl::Matrix3::Identity() * rot_stddev*rot_stddev;
    covariance.block<3,3>(3,3) = ugl::Matrix3::Identity() * vel_stddev*vel_stddev;
    covariance.block<3,3>(6,6) = ugl::Matrix3::Identity() * pos_stddev*pos_stddev;

    return covariance;
}();

const ugl::Vector<9> OffsetGenerator::s_default_uniform_range = []() {
    constexpr double rot_range = 1.5;  // [rad]
    constexpr double vel_range = 3.0;  // [m/s]
    constexpr double pos_range = 3.0;  // [m]

    ugl::Vector<9> range{};
    range.segment<3>(0) = ugl::Vector3::Ones() * rot_range;
    range.segment<3>(3) = ugl::Vector3::Ones() * vel_range;
    range.segment<3>(6) = ugl::Vector3::Ones() * pos_range;

    return range;
}();

OffsetGenerator::OffsetGenerator(const ugl::Matrix<9,9>& covariance)
    : m_gaussian(covariance)
{
}

ugl::lie::ExtendedPose OffsetGenerator::sample_gaussian() const
{
    return ugl::lie::ExtendedPose::exp(m_gaussian.sample());
}

ugl::lie::ExtendedPose OffsetGenerator::sample_uniform() const
{
    return ugl::lie::ExtendedPose::exp(m_uniform.sample());
}

} // namespace invariant::test
