#include <algorithm>
#include <iostream>
#include <fstream>
#include <string>

#include <gtest/gtest.h>

#include "iekf.h"
#include "mekf.h"

#include "accuracy_test.h"
#include "accuracy_test_config.h"

namespace invariant::test
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

template<typename FilterType>
class DataGenerationTest: public AccuracyTest<FilterType>
{
protected:
    void run_test()
    {
        const auto sensor_events = generate_events(this->trajectory_, this->sensors_);
        const auto result = this->compute_accuracy(sensor_events);

        this->RecordProperty("PositionRMSE", std::to_string(result.position_rmse));
        this->RecordProperty("VelocityRMSE", std::to_string(result.velocity_rmse));
        this->RecordProperty("RotationRMSE", std::to_string(result.rotation_rmse));

        std::cout << "position_rmse : " << result.position_rmse << '\n';
        std::cout << "velocity_rmse : " << result.velocity_rmse << '\n';
        std::cout << "rotation_rmse : " << result.rotation_rmse << '\n';

        save_to_file(result);
    }
};

using IekfTestSuite = DataGenerationTest<IEKF>;
TEST_P(IekfTestSuite, IekfTestCase) { run_test(); }
INSTANTIATE_TEST_CASE_P(
    GenerateCsvData,
    IekfTestSuite,
    test_configs_partial,
);

using MekfTestSuite = DataGenerationTest<MEKF>;
TEST_P(MekfTestSuite, MekfTestCase) { run_test(); }
INSTANTIATE_TEST_CASE_P(
    GenerateCsvData,
    MekfTestSuite,
    test_configs_partial,
);

} // namespace invariant::test

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
