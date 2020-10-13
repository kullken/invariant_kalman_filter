#ifndef INVARIANT_TEST_OFFSET_GENERATOR_H
#define INVARIANT_TEST_OFFSET_GENERATOR_H

#include <ugl/math/matrix.h>
#include <ugl/lie_group/extended_pose.h>
#include <ugl/random/normal_distribution.h>

namespace invariant::test
{

class OffsetGenerator
{
public:
    OffsetGenerator() = default;

    explicit
    OffsetGenerator(const ugl::Matrix<9,9>& covariance);

    ugl::lie::ExtendedPose sample() const;

private:
    ugl::random::NormalDistribution<9> m_gaussian{s_default_covariance};

    static const ugl::Matrix<9,9> s_default_covariance;
};

} // namespace invariant::test

#endif // INVARIANT_TEST_OFFSET_GENERATOR_H
