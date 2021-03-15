#include <vector>

#include <gtest/gtest.h>

#include "iekf.h"
#include "mekf.h"

#include "accuracy_test.h"
#include "offset_generator.h"
#include "rosbag_helpers.h"
#include "test_trajectories.h"

#include "virtual_sensor.h"
#include "imu_sensor_model.h"
#include "mocap_sensor_model.h"
#include "gps_sensor_model.h"

namespace invariant::test
{
namespace
{

const auto test_trajectories = testing::Values(
    TestTrajectory::stand_still(10),
    TestTrajectory::rotate_in_place(3600, 10),
    TestTrajectory::constant_velocity({0,1,0}, 10),
    TestTrajectory::start_stop({0,1,0}, 10),
    TestTrajectory::circle(360, 1, 10)
);

const auto test_sensor_models = testing::Values(
    std::vector{
        VirtualSensor{ImuSensorModel{ImuNoiseLevel::None, 100.0}},
        VirtualSensor{GpsSensorModel{GpsNoiseLevel::None, 10.0}},
    },
    std::vector{
        VirtualSensor{ImuSensorModel{ImuNoiseLevel::None, 100.0}},
        VirtualSensor{GpsSensorModel{GpsNoiseLevel::Low, 10.0}},
    },
    std::vector{
        VirtualSensor{ImuSensorModel{ImuNoiseLevel::Low, 100.0}},
        VirtualSensor{GpsSensorModel{GpsNoiseLevel::None, 10.0}},
    },
    std::vector{
        VirtualSensor{ImuSensorModel{ImuNoiseLevel::Low, 100.0}},
        VirtualSensor{GpsSensorModel{GpsNoiseLevel::Low, 10.0}},
    }
);

const auto offset_generators = testing::Values(
    OffsetGenerator{}
);

const auto test_configs = testing::Combine(
    test_trajectories,
    test_sensor_models,
    offset_generators
);

} // namespace

class DataGenerationTest: public AccuracyTest
{
private:
    static constexpr int kNumOffsetSamples = 10;

protected:
    void run_test()
    {
        const auto sensor_events = generate_events(this->trajectory_, this->sensors_);
        std::vector<Result> iekf_results{};
        std::vector<Result> mekf_results{};
        for (int i = 0; i < kNumOffsetSamples; ++i)
        {
            const auto initial_error = this->offset_.sample_uniform();
            const auto initial_state = this->trajectory_.get_extended_pose(0.0) * initial_error;
            const auto initial_covar = this->offset_.get_covariance();

            IEKF iekf_filter{initial_state, initial_covar};
            const auto iekf_estimates = run_filter(iekf_filter, sensor_events);
            const auto iekf_result = calculate_result(this->trajectory_, iekf_estimates);
            iekf_results.push_back(iekf_result);

            MEKF mekf_filter{initial_state, initial_covar};
            const auto mekf_estimates = run_filter(mekf_filter, sensor_events);
            const auto mekf_result = calculate_result(this->trajectory_, mekf_estimates);
            mekf_results.push_back(mekf_result);
        }
        save_to_rosbag(iekf_results, mekf_results, sensor_events);
    }
};

using RosbagTestSuite = DataGenerationTest;
TEST_P(RosbagTestSuite, TestCase) { run_test(); }
INSTANTIATE_TEST_CASE_P(
    GenerateRosbagData,
    RosbagTestSuite,
    test_configs,
);

} // namespace invariant::test

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
