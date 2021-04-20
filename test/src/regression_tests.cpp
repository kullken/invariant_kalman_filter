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

static const VirtualSensor kPerfectImu   = ImuSensorModel{ImuNoiseLevel::None, 100.0};
static const VirtualSensor kNoisyImu     = ImuSensorModel{ImuNoiseLevel::Low, 100.0};

static const VirtualSensor kPerfectGps   = GpsSensorModel{GpsNoiseLevel::None, 10.0};
static const VirtualSensor kNoisyGps     = GpsSensorModel{GpsNoiseLevel::Low, 10.0};

static const VirtualSensor kPerfectMocap = MocapSensorModel{MocapNoiseLevel::None, 10.0};
static const VirtualSensor kNoisyMocap   = MocapSensorModel{MocapNoiseLevel::Low, 10.0};

const auto test_sensor_models_gps = testing::Values(
    std::vector{kPerfectImu, kPerfectGps},
    std::vector{kPerfectImu, kNoisyGps},
    std::vector{kNoisyImu,   kPerfectGps},
    std::vector{kNoisyImu,   kNoisyGps}
);

const auto test_sensor_models_mocap = testing::Values(
    std::vector{kPerfectImu, kPerfectMocap},
    std::vector{kPerfectImu, kNoisyMocap},
    std::vector{kNoisyImu,   kPerfectMocap},
    std::vector{kNoisyImu,   kNoisyMocap}
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

using IekfMocapTestSuite = RegressionTest<IEKF>;
TEST_P(IekfMocapTestSuite, IekfTestCase) { run_test(); }
INSTANTIATE_TEST_CASE_P(
    RegressionTests,
    IekfMocapTestSuite,
    testing::Combine(
        test_trajectories,
        test_sensor_models_mocap,
        offset_generators
    ),
);

using IekfGpsTestSuite = RegressionTest<IEKF>;
TEST_P(IekfGpsTestSuite, IekfTestCase) { run_test(); }
INSTANTIATE_TEST_CASE_P(
    RegressionTests,
    IekfGpsTestSuite,
    testing::Combine(
        test_trajectories,
        test_sensor_models_gps,
        offset_generators
    ),
);

using MekfGpsTestSuite = RegressionTest<MEKF>;
TEST_P(MekfGpsTestSuite, MekfTestCase) { run_test(); }
INSTANTIATE_TEST_CASE_P(
    RegressionTests,
    MekfGpsTestSuite,
    testing::Combine(
        test_trajectories,
        test_sensor_models_gps,
        offset_generators
    ),
);

} // namespace invariant::test

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
