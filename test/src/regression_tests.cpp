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
    TestTrajectory::stand_still(10),
    TestTrajectory::rotate_in_place(3600, 10),
    TestTrajectory::constant_velocity({1,1,0}, 10),
    TestTrajectory::constant_velocity({0,0,1}, 10),
    TestTrajectory::start_stop({1,1,0}, 10),
    TestTrajectory::start_stop({0,0,1}, 10),
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
    OffsetGenerator{
        []() {
            const ugl::Vector3 rotation_stddev{0.2, 0.2, 1.0}; // [rad]
            constexpr double kVelocityStddev = 0.2;  // [m/s]
            constexpr double kPositionStddev = 0.2;  // [m]
            ugl::Matrix<9,9> covariance = ugl::Matrix<9,9>::Zero();
            covariance.block<3,3>(0,0).diagonal() = (rotation_stddev * rotation_stddev.transpose()).diagonal();
            covariance.block<3,3>(3,3) = ugl::Matrix3::Identity() * kVelocityStddev*kVelocityStddev;
            covariance.block<3,3>(6,6) = ugl::Matrix3::Identity() * kPositionStddev*kPositionStddev;
            return covariance;
        }()
    }
);

const auto test_configs = testing::Combine(
    test_trajectories,
    test_sensor_models,
    offset_generators
);

} // namepsace

template<typename FilterType>
class RegressionTest: public AccuracyTest
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
            const auto sensor_events = generate_events(this->m_trajectory, this->m_sensors);
            for (int j = 0; j < kNumOffsetSamples; ++j)
            {
                const auto result = this->compute_accuracy<FilterType>(sensor_events);
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
