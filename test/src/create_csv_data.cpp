#include <algorithm>
#include <ostream>
#include <fstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <ugl/math/vector.h>
#include <ugl/math/quaternion.h>
#include <ugl/lie_group/rotation.h>

#include "iekf.h"
#include "mekf.h"

#include "accuracy_test.h"
#include "accuracy_test_config.h"

namespace invariant::test
{

std::ostream& operator<<(std::ostream& os, const Result& result)
{
    constexpr auto delimiter = ' ';
    os << "time" << delimiter
       << "pos_err" << delimiter << "vel_err" << delimiter << "rot_err" << delimiter
       << "px_pred" << delimiter << "py_pred" << delimiter << "pz_pred" << delimiter
       << "vx_pred" << delimiter << "vy_pred" << delimiter << "vz_pred" << delimiter
       << "rx_pred" << delimiter << "ry_pred" << delimiter << "rz_pred" << delimiter
       << "px_true" << delimiter << "py_true" << delimiter << "pz_true" << delimiter
       << "vx_true" << delimiter << "vy_true" << delimiter << "vz_true" << delimiter
       << "rx_true" << delimiter << "ry_true" << delimiter << "rz_true" << delimiter
       << '\n';

    auto write_vector = [&](const ugl::Vector3& vec) {
        os << vec.x() << delimiter << vec.y() << delimiter << vec.z() << delimiter;
    };

    const auto size = result.times.size();
    for (std::size_t i = 0; i < size; ++i)
    {
        os << result.times[i] << delimiter
           << result.position_errors[i] << delimiter
           << result.velocity_errors[i] << delimiter
           << result.rotation_errors[i] << delimiter;

        write_vector(result.estimates[i].position());
        write_vector(result.estimates[i].velocity());
        write_vector(ugl::lie::SO3::log(result.estimates[i].rotation()));

        write_vector(result.ground_truth[i].position());
        write_vector(result.ground_truth[i].velocity());
        write_vector(ugl::lie::SO3::log(result.ground_truth[i].rotation()));

        os << '\n';
    }

    return os;
}

void save_to_file(const std::vector<Result>& results)
{
    auto test_info = testing::UnitTest::GetInstance()->current_test_info();
    std::string file_name = test_info->name();
    std::replace(std::begin(file_name), std::end(file_name), '/', '_');
	const std::string result_path{"/home/vk/mav_ws/src/invariant_kalman_filter/test/results/data/" + file_name + ".csv"};

    std::ofstream csv_file{result_path};
    csv_file << "# " << test_info->name() << ": " << test_info->value_param() << '\n';
    csv_file << '\n';
    csv_file << "test_case_count" << '\n' << results.size() << '\n';
    csv_file << "rows_per_case" << '\n' << results[0].times.size() << '\n';
    csv_file << '\n';

    for (const auto& result : results)
    {
        csv_file << result << '\n';
    }
}

template<typename FilterType>
class DataGenerationTest: public AccuracyTest<FilterType>
{
private:
    static constexpr int kNumOffsetSamples = 10;

protected:
    void run_test()
    {
        std::vector<Result> results{};
        for (int i = 0; i < kNumOffsetSamples; ++i)
        {
            const auto sensor_events = generate_events(this->trajectory_, this->sensors_);
            const auto result = this->compute_accuracy(sensor_events);
            results.push_back(result);
        }
        save_to_file(results);
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
