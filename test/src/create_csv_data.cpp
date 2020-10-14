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

void save_to_file(const Result& result)
{
    auto test_info = testing::UnitTest::GetInstance()->current_test_info();
    std::string file_name = test_info->name();
    std::replace(std::begin(file_name), std::end(file_name), '/', '_');
	const std::string result_path{"/home/vk/mav_ws/src/invariant_kalman_filter/test/results/data/" + file_name + ".csv"};

    std::ofstream csv_file{result_path};
    csv_file << "# " << test_info->name() << ": " << test_info->value_param() << '\n';
    csv_file << result;
}

TEST_P(IekfTestSuite, IekfTestCase)
{
    const auto sensor_events = generate_events(trajectory_, sensors_);
    const auto result = compute_accuracy(sensor_events);

    RecordProperty("PositionRMSE", std::to_string(result.position_rmse));
    RecordProperty("VelocityRMSE", std::to_string(result.velocity_rmse));
    RecordProperty("RotationRMSE", std::to_string(result.rotation_rmse));

    std::cout << "position_rmse : " << result.position_rmse << '\n';
    std::cout << "velocity_rmse : " << result.velocity_rmse << '\n';
    std::cout << "rotation_rmse : " << result.rotation_rmse << '\n';

    save_to_file(result);
}

INSTANTIATE_TEST_CASE_P(
    GenerateCsvData,
    IekfTestSuite,
    test_configs_partial,
);

TEST_P(MekfTestSuite, MekfTestCase)
{
    const auto sensor_events = generate_events(trajectory_, sensors_);
    const auto result = compute_accuracy(sensor_events);

    RecordProperty("PositionRMSE", std::to_string(result.position_rmse));
    RecordProperty("VelocityRMSE", std::to_string(result.velocity_rmse));
    RecordProperty("RotationRMSE", std::to_string(result.rotation_rmse));

    std::cout << "position_rmse : " << result.position_rmse << '\n';
    std::cout << "velocity_rmse : " << result.velocity_rmse << '\n';
    std::cout << "rotation_rmse : " << result.rotation_rmse << '\n';

    save_to_file(result);
}

INSTANTIATE_TEST_CASE_P(
    GenerateCsvData,
    MekfTestSuite,
    test_configs_partial,
);

} // namespace
} // namespace invariant::test

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
