#include <algorithm>
#include <ostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <rosbag/bag.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>

#include <ugl_ros/convert_tf2.h>

#include <ugl/math/vector.h>
#include <ugl/lie_group/rotation.h>
#include <ugl/lie_group/extended_pose.h>
#include <ugl/random/seed.h>

#include "iekf.h"
#include "mekf.h"

#include "accuracy_test.h"
#include "test_trajectories.h"
#include "offset_generator.h"

#include "virtual_sensor.h"
#include "sensor_event.h"
#include "imu_sensor_model.h"
#include "mocap_sensor_model.h"
#include "gps_sensor_model.h"

namespace invariant::test
{
namespace // rosbag
{

ros::Time to_rosbag_time(double t)
{
    // For some reason it is not allowed to add a timestamp of exactly 0 to a rosbag.
    return t == 0.0 ? ros::TIME_MIN : ros::Time{t};
}

void write_states(rosbag::Bag& rosbag,
        const std::vector<ugl::lie::ExtendedPose>& states,
        const std::vector<double>& times,
        const std::string& topic_prefix)
{
    const std::string pose_topic = topic_prefix + "/pose";
    const std::string velocity_topic = topic_prefix + "/velocity";
    const std::string log_rotation_topic = topic_prefix + "/log_rotation";

    const auto size = times.size();
    for (std::size_t i = 0; i < size; ++i)
    {
        std_msgs::Header header{};
        header.frame_id = "map";
        header.stamp = to_rosbag_time(times[i]);

        geometry_msgs::PoseStamped pose{};
        pose.header = header;
        tf2::toMsg(states[i].rotation(), pose.pose.orientation);
        tf2::toMsg(states[i].position(), pose.pose.position);
        rosbag.write(pose_topic, header.stamp, pose);

        geometry_msgs::Vector3Stamped velocity{};
        velocity.header = header;
        tf2::toMsg(states[i].velocity(), velocity.vector);
        rosbag.write(velocity_topic, header.stamp, velocity);

        geometry_msgs::Vector3Stamped log_rotation{};
        log_rotation.header = header;
        tf2::toMsg(ugl::lie::SO3::log(states[i].rotation()), log_rotation.vector);
        rosbag.write(log_rotation_topic, header.stamp, log_rotation);
    }
}

void write_errors(rosbag::Bag& rosbag,
        const std::vector<double>& errors,
        const std::vector<double>& times,
        const std::string& topic)
{
    const auto size = times.size();
    for (std::size_t i = 0; i < size; ++i)
    {
        const ros::Time stamp = to_rosbag_time(times[i]);
        std_msgs::Float64 error{};
        error.data = errors[i];
        rosbag.write(topic, stamp, error);
    }
}

void write_results(rosbag::Bag& rosbag, const std::vector<Result>& results, const std::string& topic_prefix)
{
    const auto size = results.size();
    for (std::size_t i = 0; i < size; ++i)
    {
        const auto& result = results[i];
        const std::string id = topic_prefix + "/offset_" + std::to_string(i);
        write_states(rosbag, result.estimates, result.times, id + "/estimate");
        write_errors(rosbag, result.position_errors, result.times, id + "/error/position");
        write_errors(rosbag, result.velocity_errors, result.times, id + "/error/velocity");
        write_errors(rosbag, result.rotation_errors, result.times, id + "/error/rotation");
    }
}

void save_to_rosbag(const std::vector<Result>& iekf_results, const std::vector<Result>& mekf_results, const std::vector<SensorEvent>& events)
{
    auto test_info = testing::UnitTest::GetInstance()->current_test_info();
    std::string file_name = test_info->name();
    std::replace(std::begin(file_name), std::end(file_name), '/', '_');
	const std::string result_path{"/home/vk/mav_ws/src/invariant_kalman_filter/test/results/bags/" + file_name + ".bag"};
    rosbag::Bag rosbag{result_path, rosbag::bagmode::Write};

    std_msgs::String value_param{};
    value_param.data = test_info->value_param();
    rosbag.write("/value_param", to_rosbag_time(0), value_param);

    write_states(rosbag, iekf_results[0].ground_truth, iekf_results[0].times, "/ground_truth");

    write_results(rosbag, iekf_results, "/iekf");
    write_results(rosbag, mekf_results, "/mekf");

    for (const auto& event: events)
    {
        event.write_to_rosbag(rosbag, "");
    }
}

} // namespace // rosbag

namespace // csv
{

constexpr auto delimiter = ' ';

std::ostream& operator<<(std::ostream& os, const Result& result)
{
    os << "time" << delimiter
       << "pos_err" << delimiter << "vel_err" << delimiter << "rot_err" << delimiter
       << "pos_x" << delimiter << "pos_y" << delimiter << "pos_z" << delimiter
       << "vel_x" << delimiter << "vel_y" << delimiter << "vel_z" << delimiter
       << "rot_x" << delimiter << "rot_y" << delimiter << "rot_z" << delimiter
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

        os << '\n';
    }

    return os;
}

void write_ground_truth(std::ostream& os, const std::vector<double>& times, const std::vector<ugl::lie::ExtendedPose>& states)
{
    os << "time" << delimiter
       << "pos_x" << delimiter << "pos_y" << delimiter << "pos_z" << delimiter
       << "vel_x" << delimiter << "vel_y" << delimiter << "vel_z" << delimiter
       << "rot_x" << delimiter << "rot_y" << delimiter << "rot_z" << delimiter
       << '\n';

    auto write_vector = [&](const ugl::Vector3& vec) {
        os << vec.x() << delimiter << vec.y() << delimiter << vec.z() << delimiter;
    };

    const auto size = times.size();
    for (std::size_t i = 0; i < size; ++i)
    {
        os << times[i] << delimiter;
        write_vector(states[i].position());
        write_vector(states[i].velocity());
        write_vector(ugl::lie::SO3::log(states[i].rotation()));
        os << '\n';
    }
}

void save_to_csv(const std::vector<Result>& results)
{
    auto test_info = testing::UnitTest::GetInstance()->current_test_info();

    std::stringstream ss;
    ss << "# " << test_info->name() << ": " << test_info->value_param() << '\n';
    ss << '\n';
    ss << "test_case_count" << '\n' << results.size() << '\n';
    ss << "rows_per_case" << '\n' << results[0].times.size() << '\n';
    ss << '\n';

    write_ground_truth(ss, results[0].times, results[0].ground_truth);
    ss << '\n';

    for (const auto& result : results)
    {
        ss << result << '\n';
    }

    std::string file_name = test_info->name();
    std::replace(std::begin(file_name), std::end(file_name), '/', '_');
	const std::string result_path{"/home/vk/mav_ws/src/invariant_kalman_filter/test/results/data/" + file_name + ".csv"};
    std::ofstream csv_file{result_path};
    csv_file << ss.rdbuf();
}

} // namespace // csv

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

const auto offset_generators = testing::Values(
    OffsetGenerator{
        []() {
            constexpr double kRotationStddev = 1.0;  // [rad]
            constexpr double kVelocityStddev = 0.2;  // [m/s]
            constexpr double kpositionStddev = 0.2;  // [m]
            ugl::Matrix<9,9> covariance = ugl::Matrix<9,9>::Zero();
            covariance.block<3,3>(0,0) = ugl::Matrix3::Identity() * kRotationStddev*kRotationStddev;
            covariance.block<3,3>(3,3) = ugl::Matrix3::Identity() * kVelocityStddev*kVelocityStddev;
            covariance.block<3,3>(6,6) = ugl::Matrix3::Identity() * kpositionStddev*kpositionStddev;
            return covariance;
        }()
    }
);

class GenerateData: public testing::TestWithParam<std::tuple<std::vector<VirtualSensor>, OffsetGenerator>>
{
private:
    static constexpr int kNumOffsetSamples = 10;

protected:
    GenerateData()
        : m_sensors(std::get<0>(GetParam()))
        , m_offset(std::get<1>(GetParam()))
    {
        ugl::random::set_seed(117);
    }

    void run_test(const TestTrajectory& test_trajectory)
    {
        const auto& trajectory = test_trajectory.traj;
        const auto sensor_events = generate_events(trajectory, this->m_sensors);
        std::vector<Result> iekf_results{};
        std::vector<Result> mekf_results{};
        for (int i = 0; i < kNumOffsetSamples; ++i)
        {
            const auto initial_error = this->m_offset.sample_uniform();
            const auto initial_state = trajectory.get_extended_pose(0.0) * initial_error;
            const auto initial_covar = this->m_offset.get_covariance();

            IEKF iekf_filter{initial_state, initial_covar};
            const auto iekf_estimates = run_filter(iekf_filter, sensor_events);
            const auto iekf_result = calculate_result(trajectory, iekf_estimates);
            iekf_results.push_back(iekf_result);

            MEKF mekf_filter{initial_state, initial_covar};
            const auto mekf_estimates = run_filter(mekf_filter, sensor_events);
            const auto mekf_result = calculate_result(trajectory, mekf_estimates);
            mekf_results.push_back(mekf_result);
        }
        // save_to_rosbag(iekf_results, mekf_results, sensor_events);
        save_to_csv(iekf_results);
    }

protected:
    std::vector<VirtualSensor> m_sensors;
    OffsetGenerator m_offset;
};


TEST_P(GenerateData, CircleTest)
{
    run_test(TestTrajectory{"Circle: 360, 1m, 10s", circle(360, 1, 10)});
}
TEST_P(GenerateData, CircleTest2m)
{
    run_test(TestTrajectory{"Circle: 360, 2m, 10s", circle(360, 2, 10)});
}
INSTANTIATE_TEST_CASE_P(
    Visualization,
    GenerateData,
    ::testing::Combine(test_sensor_models, offset_generators),
);

} // namespace invariant::test

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
