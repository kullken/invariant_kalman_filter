#include <algorithm>
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

void save_to_file(const Result& result)
{
    auto test_info = testing::UnitTest::GetInstance()->current_test_info();
    std::string file_name = test_info->name();
    std::replace(std::begin(file_name), std::end(file_name), '/', '_');
	const std::string result_path{"/home/vk/mav_ws/src/invariant_kalman_filter/test/results/" + file_name + ".csv"};

    std::ofstream csv_file{result_path};
    csv_file << "# " << test_info->name() << ": " << test_info->value_param() << '\n';
    csv_file << result;
}

TEST_P(IekfTestSuite, IekfTestCase)
{
    const auto result = compute_accuracy();

    RecordProperty("PositionRMSE", std::to_string(result.position_rmse));
    RecordProperty("VelocityRMSE", std::to_string(result.velocity_rmse));
    RecordProperty("RotationRMSE", std::to_string(result.rotation_rmse));

    std::cout << "result.position_rmse : " << result.position_rmse << '\n';
    std::cout << "result.velocity_rmse : " << result.velocity_rmse << '\n';
    std::cout << "result.rotation_rmse : " << result.rotation_rmse << '\n';

    save_to_file(result);
}

INSTANTIATE_TEST_CASE_P(
    AccuracyTestBase,
    IekfTestSuite,
    test_configs,
);

} // namespace
} // namespace invariant::test

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
