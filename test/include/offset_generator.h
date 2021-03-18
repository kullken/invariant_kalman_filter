#ifndef INVARIANT_TEST_OFFSET_GENERATOR_H
#define INVARIANT_TEST_OFFSET_GENERATOR_H

#include <ugl/math/vector.h>
#include <ugl/math/matrix.h>
#include <ugl/lie_group/extended_pose.h>
#include <ugl/random/normal_distribution.h>
#include <ugl/random/uniform_distribution.h>

namespace invariant::test
{

class OffsetGenerator
{
public:
    OffsetGenerator() : OffsetGenerator(ugl::Matrix<9,9>::Identity()) {}

    explicit
    OffsetGenerator(const ugl::Matrix<9,9>& covariance);

    ugl::lie::ExtendedPose sample_gaussian() const;

    ugl::lie::ExtendedPose sample_uniform() const;

    const ugl::Matrix<9,9>& get_covariance() const;

private:
    ugl::random::NormalDistribution<9> m_gaussian;
    ugl::random::UniformDistribution<9> m_uniform;
};

} // namespace invariant::test

#endif // INVARIANT_TEST_OFFSET_GENERATOR_H
