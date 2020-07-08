#include <iostream>
#include <fstream>
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

TEST_P(IekfTestSuite, IekfTestCase)
{
    const AccuracyTest::Result result = compute_accuracy(filter_, trajectory_);

    RecordProperty("PositionRMSE", std::to_string(result.position_rmse));
    RecordProperty("VelocityRMSE", std::to_string(result.velocity_rmse));
    RecordProperty("RotationRMSE", std::to_string(result.rotation_rmse));

    std::cout << "result.position_rmse : " << result.position_rmse << '\n';
    std::cout << "result.velocity_rmse : " << result.velocity_rmse << '\n';
    std::cout << "result.rotation_rmse : " << result.rotation_rmse << '\n';

	const std::string result_path{"/home/vk/mav_ws/src/invariant_kalman_filter/test/results/prototype_data.csv"};

    std::ofstream csv_file{result_path};
    csv_file << result;
}

INSTANTIATE_TEST_CASE_P(
    AccuracyTestBase,
    IekfTestSuite,
    test_configs
);

} // namespace
} // namespace invariant::test

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
