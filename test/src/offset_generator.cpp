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

ugl::lie::ExtendedPose OffsetGenerator::sample_uniform() const
{
    return ugl::lie::ExtendedPose::exp(m_uniform.sample());
}

const ugl::Matrix<9,9>& OffsetGenerator::get_covariance() const
{
    return m_gaussian.covar();
}

} // namespace invariant::test
