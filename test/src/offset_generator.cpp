#include "offset_generator.h"

#include <cmath>

#include <ugl/math/vector.h>
#include <ugl/math/matrix.h>
#include <ugl/lie_group/extended_pose.h>

namespace invariant::test
{

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
