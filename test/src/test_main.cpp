#include <string>

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
    RecordProperty("PositionRMSE", std::to_string(result.position_rmse));
    RecordProperty("VelocityRMSE", std::to_string(result.velocity_rmse));
    RecordProperty("RotationRMSE", std::to_string(result.rotation_rmse));
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
