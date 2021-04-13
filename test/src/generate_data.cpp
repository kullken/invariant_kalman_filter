#include <tuple>
#include <vector>

#include <gtest/gtest.h>

#include <ugl/math/matrix.h>

#include "iekf.h"
#include "mekf.h"

#include "accuracy_test.h"
#include "csv_helpers.h"
#include "offset_generator.h"
#include "rosbag_helpers.h"
#include "test_trajectories.h"

#include "virtual_sensor.h"
#include "imu_sensor_model.h"
#include "mocap_sensor_model.h"
#include "gps_sensor_model.h"

namespace invariant::test
{

static constexpr double kImuFrequency   = 100.0;
static constexpr double kGpsFrequency   = 2.0;
static constexpr double kMocapFrequency = 2.0;

static const ugl::Matrix<6,6> kImuNoise = []() {
    constexpr double kGyroStdev = 0.1;  // [rad/s]
    constexpr double kAccelStddev = 2.5;  // [m/s^2]
    ugl::Matrix<6,6> covariance = ugl::Matrix<6,6>::Zero();
    covariance.block<3,3>(0,0) = ugl::Matrix3::Identity() * kGyroStdev*kGyroStdev;
    covariance.block<3,3>(3,3) = ugl::Matrix3::Identity() * kAccelStddev*kAccelStddev;
    return covariance;
}();

static const ugl::Matrix<6,6> kPoseNoise = []() {
    constexpr double kRotationStddev = 0.1;  // [rad]
    constexpr double kPositionStddev = 0.1;  // [m]
    ugl::Matrix<6,6> covariance = ugl::Matrix<6,6>::Zero();
    covariance.block<3,3>(0,0) = ugl::Matrix3::Identity() * kRotationStddev*kRotationStddev;
    covariance.block<3,3>(3,3) = ugl::Matrix3::Identity() * kPositionStddev*kPositionStddev;
    return covariance;
}();

static const ugl::Matrix3 kPositionNoise = []() {
    constexpr double kPositionStddev = 0.1;  // [m]
    return ugl::Matrix3::Identity() * kPositionStddev*kPositionStddev;
}();

static const VirtualSensor kPerfectImu   = ImuSensorModel{ugl::Matrix<6,6>::Zero(), kImuNoise, kImuFrequency};
static const VirtualSensor kNoisyImu     = ImuSensorModel{kImuNoise, kImuNoise, kImuFrequency};

static const VirtualSensor kPerfectMocap = MocapSensorModel{ugl::Matrix<6,6>::Zero(), kPoseNoise, kMocapFrequency};
static const VirtualSensor kNoisyMocap   = MocapSensorModel{kPoseNoise, kPoseNoise, kMocapFrequency};

static const VirtualSensor kPerfectGps   = GpsSensorModel{ugl::Matrix3::Zero(), kPositionNoise, kGpsFrequency};
static const VirtualSensor kNoisyGps     = GpsSensorModel{kPositionNoise, kPositionNoise, kGpsFrequency};

static const auto kTestSensorModelsMocap = testing::Values(
    std::vector{kPerfectImu, kPerfectMocap},
    std::vector{kPerfectImu, kNoisyMocap},
    std::vector{kNoisyImu,   kPerfectMocap},
    std::vector{kNoisyImu,   kNoisyMocap}
);

static const auto kTestSensorModelsGps = testing::Values(
    std::vector{kPerfectImu, kPerfectGps},
    std::vector{kPerfectImu, kNoisyGps},
    std::vector{kNoisyImu,   kPerfectGps},
    std::vector{kNoisyImu,   kNoisyGps}
);

static const ugl::Matrix<9,9> kInitialCovariance = []() {
    const ugl::Vector3 rotation_stddev{0.1, 0.1, 1.0}; // [rad]
    constexpr double kVelocityStddev = 0.2;  // [m/s]
    constexpr double kPositionStddev = 0.2;  // [m]
    ugl::Matrix<9,9> covariance = ugl::Matrix<9,9>::Zero();
    covariance.block<3,3>(0,0).diagonal() = (rotation_stddev * rotation_stddev.transpose()).diagonal();
    covariance.block<3,3>(3,3) = ugl::Matrix3::Identity() * kVelocityStddev*kVelocityStddev;
    covariance.block<3,3>(6,6) = ugl::Matrix3::Identity() * kPositionStddev*kPositionStddev;
    return covariance;
}();

static const auto kRandomInitialOffsets = testing::Values(
    OffsetGenerator{kInitialCovariance}.sample_uniform(10)
);

static const auto kYawSpreadInitialOffsets = testing::Values(
    InitialValue::uniform_yaw_spread(1.5, 9, kInitialCovariance)
);

class GenerateData: public testing::TestWithParam<std::tuple<std::vector<VirtualSensor>, std::vector<InitialValue>>>
{
protected:
    GenerateData()
        : m_sensors(std::get<0>(GetParam()))
        , m_initial_values(std::get<1>(GetParam()))
    {
        ugl::random::set_seed(117);
    }

    void run_test(const TestTrajectory& test_trajectory)
    {
        const auto& trajectory = test_trajectory.traj;
        const auto sensor_events = generate_events(trajectory, m_sensors);
        std::vector<Result> iekf_results{};
        std::vector<Result> mekf_results{};
        for (const auto& initial_value: m_initial_values)
        {
            const auto initial_state = get_initial_state(trajectory, initial_value);

            IEKF iekf_filter{initial_state, initial_value.covariance};
            const auto iekf_estimates = run_filter(iekf_filter, sensor_events);
            const auto iekf_result = calculate_result(trajectory, iekf_estimates);
            iekf_results.push_back(iekf_result);

            MEKF mekf_filter{initial_state, initial_value.covariance};
            const auto mekf_estimates = run_filter(mekf_filter, sensor_events);
            const auto mekf_result = calculate_result(trajectory, mekf_estimates);
            mekf_results.push_back(mekf_result);
        }
        save_to_rosbag(iekf_results, mekf_results, sensor_events);
        save_to_csv(iekf_results, "Iekf");
        save_to_csv(mekf_results, "Mekf");
    }

private:
    ugl::lie::ExtendedPose get_initial_state(const ugl::trajectory::Trajectory& trajectory, const InitialValue& initial_value)
    {
        if (initial_value.local) {
            return trajectory.get_extended_pose(0.0) * initial_value.offset;
        }
        else {
            return initial_value.offset;
        }
    }

protected:
    std::vector<VirtualSensor> m_sensors;
    std::vector<InitialValue> m_initial_values;
};

TEST_P(GenerateData, HelixTest)
{
    run_test(TestTrajectory::helix(720, 2, 0.2, 20));
}
TEST_P(GenerateData, HexagonTest)
{
    run_test(TestTrajectory::hexagon_start_stop(1, 10));
}
INSTANTIATE_TEST_CASE_P(
    Visualization,
    GenerateData,
    ::testing::Combine(
        kTestSensorModelsMocap,
        kRandomInitialOffsets
    ),
);

} // namespace invariant::test

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
