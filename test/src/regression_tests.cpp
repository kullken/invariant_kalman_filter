#include <algorithm>
#include <iostream>
#include <fstream>
#include <string>

#include <gtest/gtest.h>

#include "accuracy_test.h"
#include "accuracy_test_config.h" 

namespace invariant::test
{
namespace
{

TEST_P(IekfTestSuite, IekfTestCase)
{
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

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
