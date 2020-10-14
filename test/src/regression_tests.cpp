#include <iostream>
#include <string>

#include <gtest/gtest.h>

#include "accuracy_test.h"
#include "accuracy_test_config.h"

namespace invariant::test
{
namespace
{

constexpr int kNumTestRuns      = 5;
constexpr int kNumOffsetSamples = 10;

using IekfTestSuite = AccuracyTest<IEKF>;

TEST_P(IekfTestSuite, IekfTestCase)
{
    double position_rmse{0};
    double velocity_rmse{0};
    double rotation_rmse{0};

    for (int i = 0; i < kNumTestRuns; ++i)
    {
        const auto sensor_events = generate_events(trajectory_, sensors_);
        for (int j = 0; j < kNumOffsetSamples; ++j)
        {
            const auto result = compute_accuracy(sensor_events);
            position_rmse += result.position_rmse;
            velocity_rmse += result.velocity_rmse;
            rotation_rmse += result.rotation_rmse;
        }
    }

    position_rmse /= kNumTestRuns * kNumOffsetSamples;
    velocity_rmse /= kNumTestRuns * kNumOffsetSamples;
    rotation_rmse /= kNumTestRuns * kNumOffsetSamples;

    RecordProperty("PositionRMSE", std::to_string(position_rmse));
    RecordProperty("VelocityRMSE", std::to_string(velocity_rmse));
    RecordProperty("RotationRMSE", std::to_string(rotation_rmse));

    std::cout << "position_rmse : " << position_rmse << '\n';
    std::cout << "velocity_rmse : " << velocity_rmse << '\n';
    std::cout << "rotation_rmse : " << rotation_rmse << '\n';
}

INSTANTIATE_TEST_CASE_P(
    AccuracyTestBase,
    IekfTestSuite,
    test_configs_full,
);

using MekfTestSuite = AccuracyTest<MEKF>;

TEST_P(MekfTestSuite, MekfTestCase)
{
    double position_rmse{0};
    double velocity_rmse{0};
    double rotation_rmse{0};

    for (int i = 0; i < kNumTestRuns; ++i)
    {
        const auto sensor_events = generate_events(trajectory_, sensors_);
        for (int j = 0; j < kNumOffsetSamples; ++j)
        {
            const auto result = compute_accuracy(sensor_events);
            position_rmse += result.position_rmse;
            velocity_rmse += result.velocity_rmse;
            rotation_rmse += result.rotation_rmse;
        }
    }

    position_rmse /= kNumTestRuns * kNumOffsetSamples;
    velocity_rmse /= kNumTestRuns * kNumOffsetSamples;
    rotation_rmse /= kNumTestRuns * kNumOffsetSamples;

    RecordProperty("PositionRMSE", std::to_string(position_rmse));
    RecordProperty("VelocityRMSE", std::to_string(velocity_rmse));
    RecordProperty("RotationRMSE", std::to_string(rotation_rmse));

    std::cout << "position_rmse : " << position_rmse << '\n';
    std::cout << "velocity_rmse : " << velocity_rmse << '\n';
    std::cout << "rotation_rmse : " << rotation_rmse << '\n';
}

INSTANTIATE_TEST_CASE_P(
    AccuracyTestBase,
    MekfTestSuite,
    test_configs_full,
);

} // namespace
} // namespace invariant::test

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
