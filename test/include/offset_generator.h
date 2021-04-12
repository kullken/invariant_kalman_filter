#ifndef INVARIANT_TEST_OFFSET_GENERATOR_H
#define INVARIANT_TEST_OFFSET_GENERATOR_H

#include <ugl/math/vector.h>
#include <ugl/math/matrix.h>
#include <ugl/lie_group/extended_pose.h>
#include <ugl/random/normal_distribution.h>
#include <ugl/random/uniform_distribution.h>

namespace invariant::test
{

struct InitialValue
{
    ugl::lie::ExtendedPose offset;
    ugl::Matrix<9,9> covariance;

    bool local = true;

    /// @brief Creates a vector of InitialValue's with uniform spread of yaw values in range +-yaw_range.
    static std::vector<InitialValue> uniform_yaw_spread(double yaw_range, int sample_count, const ugl::Matrix<9,9>& initial_covariance);
};

class OffsetGenerator
{
public:
    OffsetGenerator() : OffsetGenerator(ugl::Matrix<9,9>::Identity()) {}

    explicit
    OffsetGenerator(const ugl::Matrix<9,9>& covariance);

    ugl::lie::ExtendedPose sample_gaussian() const;

    std::vector<InitialValue> sample_gaussian(int number_samples) const;

    ugl::lie::ExtendedPose sample_uniform() const;

    std::vector<InitialValue> sample_uniform(int number_samples) const;

    const ugl::Matrix<9,9>& get_covariance() const;

private:
    ugl::random::NormalDistribution<9> m_gaussian;
    ugl::random::UniformDistribution<9> m_uniform;
};

} // namespace invariant::test

#endif // INVARIANT_TEST_OFFSET_GENERATOR_H
