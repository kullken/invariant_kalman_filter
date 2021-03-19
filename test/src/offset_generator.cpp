#include "offset_generator.h"

#include <cmath>

#include <ugl/math/vector.h>
#include <ugl/math/matrix.h>
#include <ugl/math/quaternion.h>
#include <ugl/lie_group/extended_pose.h>

namespace invariant::test
{

std::vector<InitialValue> InitialValue::uniform_yaw_spread(double yaw_range, int sample_count, const ugl::Matrix<9,9>& initial_covariance)
{
    std::vector<InitialValue> initial_values{};
    const double yaw0 = -yaw_range;
    const double yaw_step = 2*yaw_range / (sample_count-1);
    for (int i = 0; i < sample_count; ++i)
    {
        const auto quat = ugl::math::to_quat(yaw0 + i*yaw_step, ugl::Vector3::UnitZ());
        const auto initial_value = ugl::lie::ExtendedPose{quat, ugl::Vector3::Zero(), ugl::Vector3::Zero()};
        initial_values.push_back({initial_value, initial_covariance});
    }
    return initial_values;
}

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

std::vector<InitialValue> OffsetGenerator::sample_gaussian(int number_samples) const
{
    std::vector<InitialValue> initial_values{};
    for (int i = 0; i < number_samples; ++i)
    {
        initial_values.push_back({sample_gaussian(), m_gaussian.covar()});
    }
    return initial_values;
}

ugl::lie::ExtendedPose OffsetGenerator::sample_uniform() const
{
    return ugl::lie::ExtendedPose::exp(m_uniform.sample());
}

std::vector<InitialValue> OffsetGenerator::sample_uniform(int number_samples) const
{
    std::vector<InitialValue> initial_values{};
    for (int i = 0; i < number_samples; ++i)
    {
        initial_values.push_back({sample_uniform(), m_gaussian.covar()});
    }
    return initial_values;
}

const ugl::Matrix<9,9>& OffsetGenerator::get_covariance() const
{
    return m_gaussian.covar();
}

} // namespace invariant::test
