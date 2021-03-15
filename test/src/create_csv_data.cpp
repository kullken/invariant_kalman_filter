#include <vector>

#include <gtest/gtest.h>

#include "iekf.h"
#include "mekf.h"

#include "accuracy_test.h"
#include "csv_helpers.h"
#include "offset_generator.h"
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
    TestTrajectory::constant_velocity({1,1,0}, 10),
    TestTrajectory::start_stop({1,1,0}, 10),
    TestTrajectory::circle(720, 1, 10)
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

template<typename FilterType>
class DataGenerationTest: public AccuracyTest
{
private:
    static constexpr int kNumOffsetSamples = 20;

protected:
    void run_test()
    {
        const auto sensor_events = generate_events(this->trajectory_, this->sensors_);
        std::vector<Result> results{};
        for (int i = 0; i < kNumOffsetSamples; ++i)
        {
            const auto result = this->compute_accuracy<FilterType>(sensor_events);
            results.push_back(result);
        }
        save_to_csv(results);
    }
};

using IekfTestSuite = DataGenerationTest<IEKF>;
TEST_P(IekfTestSuite, IekfTestCase) { run_test(); }
INSTANTIATE_TEST_CASE_P(
    GenerateCsvData,
    IekfTestSuite,
    test_configs,
);

using MekfTestSuite = DataGenerationTest<MEKF>;
TEST_P(MekfTestSuite, MekfTestCase) { run_test(); }
INSTANTIATE_TEST_CASE_P(
    GenerateCsvData,
    MekfTestSuite,
    test_configs,
);

} // namespace invariant::test

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
