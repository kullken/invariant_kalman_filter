#include <gtest/gtest.h>

#include <ugl/math/vector.h>
#include <ugl/trajectory/trajectory.h>

#include "accuracy_test.h"
#include "accuracy_test_config.h"

namespace invariant::test
{
namespace
{

TEST_P(AccuracyTestParam, noNoiseTest)
{
    AccuracyTest::Result result = compute_accuracy(filter_, trajectory_);
    RecordProperty("Accuracy", result.final_accuracy);
}

INSTANTIATE_TEST_CASE_P(
    AccuracyTestInstantiation,
    AccuracyTestParam,
    test_configs,
    [](const testing::TestParamInfo<AccuracyTestParam::ParamType> &info) { return info.param.name; });

} // namespace
} // namespace invariant::test

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
