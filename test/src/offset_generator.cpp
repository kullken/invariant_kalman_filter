#include "offset_generator.h"

#include <cmath>

#include <ugl/math/vector.h>
#include <ugl/math/matrix.h>
#include <ugl/lie_group/extended_pose.h>

namespace invariant::test
{
namespace
{

constexpr double kRotationStddev = 1.0;  // [rad]
constexpr double kVelocityStddev = 1.0;  // [m/s]
constexpr double kpositionStddev = 1.0;  // [m]

} // namespace

const ugl::Matrix<9,9> OffsetGenerator::s_default_covariance = []() {
    ugl::Matrix<9,9> covariance = ugl::Matrix<9,9>::Zero();
    covariance.block<3,3>(0,0) = ugl::Matrix3::Identity() * kRotationStddev*kRotationStddev;
    covariance.block<3,3>(3,3) = ugl::Matrix3::Identity() * kVelocityStddev*kVelocityStddev;
    covariance.block<3,3>(6,6) = ugl::Matrix3::Identity() * kpositionStddev*kpositionStddev;
    return covariance;
}();

const ugl::Vector<9> OffsetGenerator::s_default_uniform_range = []() {
    ugl::Vector<9> range{};
    range.segment<3>(0) = ugl::Vector3::Ones() * std::sqrt(3)*kRotationStddev;
    range.segment<3>(3) = ugl::Vector3::Ones() * std::sqrt(3)*kVelocityStddev;
    range.segment<3>(6) = ugl::Vector3::Ones() * std::sqrt(3)*kpositionStddev;
    return range;
}();

OffsetGenerator::OffsetGenerator(const ugl::Matrix<9,9>& covariance)
    : m_gaussian(covariance)
{
    ugl::Vector<9> range = std::sqrt(3) * covariance.diagonal().cwiseSqrt();
    m_uniform = ugl::random::UniformDistribution<9>{-range, range};
}

ugl::lie::ExtendedPose OffsetGenerator::sample_gaussian() const
{
    return ugl::lie::ExtendedPose::exp(m_gaussian.sample());
}

ugl::lie::ExtendedPose OffsetGenerator::sample_uniform() const
{
    return ugl::lie::ExtendedPose::exp(m_uniform.sample());
}

const ugl::Matrix<9,9>& OffsetGenerator::get_covariance() const
{
    return m_gaussian.covar();
}

} // namespace invariant::test
