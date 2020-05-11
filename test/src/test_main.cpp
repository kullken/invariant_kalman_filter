#include <gtest/gtest.h>

#include <ugl/math/vector.h>
#include <ugl/trajectory/trajectory.h>

#include "accuracy_test.h"

namespace invariant::test
{
namespace
{

auto test_trajectories = testing::Values(
    AccuracyTest::Config{"TestConfig_1", {}},
    AccuracyTest::Config{"TestConfig_2", {}});

TEST_P(AccuracyTestParam, noNoiseTest)
{
    AccuracyTest::Result result = compute_accuracy(iekf_, trajectory_);
    RecordProperty("Accuracy", result.final_accuracy);
}

INSTANTIATE_TEST_CASE_P(
    AccuracyTestInstantiation,
    AccuracyTestParam,
    test_trajectories,
    [](const testing::TestParamInfo<AccuracyTestParam::ParamType> &info) { return info.param.name; });

} // namespace
} // namespace invariant::test

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
