#include <iostream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "iekf.h"
#include "mekf.h"

#include "accuracy_test.h"
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
    TestTrajectory{"StandStill 10s", stand_still(10)},
    TestTrajectory{"Rotate 3600 left, 10s", rotate_in_place(3600, 10)},
    TestTrajectory{"ConstantVel xy: 10m; 10s", constant_velocity({1,1,0}, 10)},
    TestTrajectory{"ConstantVel  z: 10m; 10s", constant_velocity({0,0,1}, 10)},
    TestTrajectory{"Translate Quadratic xy: 10m; 10s", quadratic_translation({10,10,0}, 10)},
    TestTrajectory{"Translate Quadratic z: 10m; 10s", quadratic_translation({0,0,10}, 10)},
    TestTrajectory{"StartStop: {1,1,0}, 10s", start_stop({1,1,0}, 10)},
    TestTrajectory{"StartStop: {0,0,1}, 10s", start_stop({0,0,1}, 10)},
    TestTrajectory{"Circle: 720, 1m, 10s", circle(720, 1, 10)}
);

const auto test_sensor_models = testing::Values(
    std::vector{
        VirtualSensor{ImuSensorModel{ImuNoiseLevel::None, 100.0}},
        VirtualSensor{GpsSensorModel{GpsNoiseLevel::None, 20.0}},
    },
    std::vector{
        VirtualSensor{ImuSensorModel{ImuNoiseLevel::None, 100.0}},
        VirtualSensor{GpsSensorModel{GpsNoiseLevel::Low, 20.0}},
    },
    std::vector{
        VirtualSensor{ImuSensorModel{ImuNoiseLevel::Mueller18, 100.0}},
        VirtualSensor{GpsSensorModel{GpsNoiseLevel::None, 20.0}},
    },
    std::vector{
        VirtualSensor{ImuSensorModel{ImuNoiseLevel::Mueller18, 100.0}},
        VirtualSensor{GpsSensorModel{GpsNoiseLevel::Low, 20.0}},
    }
);

const auto test_configs = testing::Combine(
    test_trajectories,
    test_sensor_models
);

} // namepsace

template<typename FilterType>
class RegressionTest: public AccuracyTest<FilterType>
{
private:
    static constexpr int kNumTestRuns      = 10;
    static constexpr int kNumOffsetSamples = 10;

protected:
    void run_test()
    {
        double position_rmse{0};
        double velocity_rmse{0};
        double rotation_rmse{0};

        for (int i = 0; i < kNumTestRuns; ++i)
        {
            const auto sensor_events = generate_events(this->trajectory_, this->sensors_);
            for (int j = 0; j < kNumOffsetSamples; ++j)
            {
                const auto result = this->compute_accuracy(sensor_events);
                position_rmse += result.position_rmse;
                velocity_rmse += result.velocity_rmse;
                rotation_rmse += result.rotation_rmse;
            }
        }

        position_rmse /= kNumTestRuns * kNumOffsetSamples;
        velocity_rmse /= kNumTestRuns * kNumOffsetSamples;
        rotation_rmse /= kNumTestRuns * kNumOffsetSamples;

        this->RecordProperty("PositionRMSE", std::to_string(position_rmse));
        this->RecordProperty("VelocityRMSE", std::to_string(velocity_rmse));
        this->RecordProperty("RotationRMSE", std::to_string(rotation_rmse));

        std::cout << "position_rmse : " << position_rmse << '\n';
        std::cout << "velocity_rmse : " << velocity_rmse << '\n';
        std::cout << "rotation_rmse : " << rotation_rmse << '\n';
    }
};

using IekfTestSuite = RegressionTest<IEKF>;
TEST_P(IekfTestSuite, IekfTestCase) { run_test(); }
INSTANTIATE_TEST_CASE_P(
    RegressionTests,
    IekfTestSuite,
    test_configs,
);

using MekfTestSuite = RegressionTest<MEKF>;
TEST_P(MekfTestSuite, MekfTestCase) { run_test(); }
INSTANTIATE_TEST_CASE_P(
    RegressionTests,
    MekfTestSuite,
    test_configs,
);

} // namespace invariant::test

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
