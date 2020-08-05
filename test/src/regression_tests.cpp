#include <algorithm>
#include <iostream>
#include <fstream>
#include <string>

#include <gtest/gtest.h>

#include <ugl/math/vector.h>
#include <ugl/trajectory/trajectory.h>
#include <ugl/random/seed.h>

#include "accuracy_test.h"
#include "accuracy_test_config.h" 

namespace invariant::test
{
namespace
{

TEST_P(IekfTestSuite, IekfTestCase)
{
    ugl::random::set_seed(117);
    const auto result = compute_accuracy();

    RecordProperty("PositionRMSE", std::to_string(result.position_rmse));
    RecordProperty("VelocityRMSE", std::to_string(result.velocity_rmse));
    RecordProperty("RotationRMSE", std::to_string(result.rotation_rmse));

    std::cout << "result.position_rmse : " << result.position_rmse << '\n';
    std::cout << "result.velocity_rmse : " << result.velocity_rmse << '\n';
    std::cout << "result.rotation_rmse : " << result.rotation_rmse << '\n';
}

INSTANTIATE_TEST_CASE_P(
    AccuracyTestBase,
    IekfTestSuite,
    test_configs_full,
);

TEST_P(MekfTestSuite, MekfTestCase)
{
    ugl::random::set_seed(117);
    const Result result = compute_accuracy();

    RecordProperty("PositionRMSE", std::to_string(result.position_rmse));
    RecordProperty("VelocityRMSE", std::to_string(result.velocity_rmse));
    RecordProperty("RotationRMSE", std::to_string(result.rotation_rmse));

    std::cout << "result.position_rmse : " << result.position_rmse << '\n';
    std::cout << "result.velocity_rmse : " << result.velocity_rmse << '\n';
    std::cout << "result.rotation_rmse : " << result.rotation_rmse << '\n';
}

INSTANTIATE_TEST_CASE_P(
    AccuracyTestBase,
    MekfTestSuite,
    test_configs_full,
);

} // namespace
} // namespace invariant::test

// TODO: Move setting of random seed to constructor of test fixture.
// TODO: Write unit tests for ugl.
// TODO: Use #ifndef instead of #pragma once header guards.
// TODO?: Compute average result over ~10 runs.
// TODO?: Export and plot more data from tests.
// TODO?: Create more "realistic" test trajectories e.g. combined translation&rotation or sample different trajectories.
// TODO?: Write comparator script for two results with same format.
// TODO?: Write mocap model for MEKF or position model for IEKF.
// TODO?: Create different test configurations e.g. "full" and "partial" (e.g. smaller config for rostest).
// TODO?: Create tests in ROS environment and compare performance to fully virtual.

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
