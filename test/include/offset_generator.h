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
    OffsetGenerator() = default;

    explicit
    OffsetGenerator(const ugl::Matrix<9,9>& covariance);

    ugl::lie::ExtendedPose sample_gaussian() const;

    ugl::lie::ExtendedPose sample_uniform() const;

    const ugl::Matrix<9,9>& get_covariance() const;

private:
    ugl::random::NormalDistribution<9> m_gaussian{s_default_covariance};
    ugl::random::UniformDistribution<9> m_uniform{-s_default_uniform_range, s_default_uniform_range};

    static const ugl::Matrix<9,9> s_default_covariance;
    static const ugl::Vector<9> s_default_uniform_range;
};

} // namespace invariant::test

#endif // INVARIANT_TEST_OFFSET_GENERATOR_H
