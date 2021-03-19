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

const auto test_sensor_models = testing::Values(
    std::vector{
        VirtualSensor{ImuSensorModel{ImuNoiseLevel::None, 100.0}},
        VirtualSensor{GpsSensorModel{GpsNoiseLevel::None, 2.0}},
    },
    std::vector{
        VirtualSensor{ImuSensorModel{ImuNoiseLevel::None, 100.0}},
        VirtualSensor{GpsSensorModel{GpsNoiseLevel::Low, 2.0}},
    },
    std::vector{
        VirtualSensor{ImuSensorModel{ImuNoiseLevel::Low, 100.0}},
        VirtualSensor{GpsSensorModel{GpsNoiseLevel::None, 2.0}},
    },
    std::vector{
        VirtualSensor{ImuSensorModel{ImuNoiseLevel::Low, 100.0}},
        VirtualSensor{GpsSensorModel{GpsNoiseLevel::Low, 2.0}},
    }
);

static const ugl::Matrix<9,9> s_initial_covariance = []() {
    const ugl::Vector3 rotation_stddev{0.2, 0.2, 1.0}; // [rad]
    constexpr double kVelocityStddev = 0.2;  // [m/s]
    constexpr double kpositionStddev = 0.2;  // [m]
    ugl::Matrix<9,9> covariance = ugl::Matrix<9,9>::Zero();
    covariance.block<3,3>(0,0).diagonal() = (rotation_stddev * rotation_stddev.transpose()).diagonal();
    covariance.block<3,3>(3,3) = ugl::Matrix3::Identity() * kVelocityStddev*kVelocityStddev;
    covariance.block<3,3>(6,6) = ugl::Matrix3::Identity() * kpositionStddev*kpositionStddev;
    return covariance;
}();

const auto initial_offsets = testing::Values(
    OffsetGenerator{s_initial_covariance}.sample_uniform(10)
);

const auto yaw_spread_initial_offsets = testing::Values(
    InitialValue::uniform_yaw_spread(1.5, 9, s_initial_covariance)
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
            const auto initial_state = trajectory.get_extended_pose(0.0) * initial_value.offset;

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

protected:
    std::vector<VirtualSensor> m_sensors;
    std::vector<InitialValue> m_initial_values;
};


TEST_P(GenerateData, CircleTest)
{
    run_test(TestTrajectory::circle(360, 1, 10));
}
TEST_P(GenerateData, HelixTest)
{
    run_test(TestTrajectory::helix(720, 2, 0.5, 20));
}
TEST_P(GenerateData, HexagonTest)
{
    run_test(TestTrajectory::hexagon_start_stop(1, 10));
}
INSTANTIATE_TEST_CASE_P(
    Visualization,
    GenerateData,
    ::testing::Combine(
        test_sensor_models,
        initial_offsets
    ),
);

} // namespace invariant::test

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
