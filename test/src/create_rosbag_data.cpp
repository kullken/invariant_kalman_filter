#include <algorithm>
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

#include <ugl/lie_group/rotation.h>
#include <ugl/lie_group/extended_pose.h>

#include "iekf.h"
#include "mekf.h"

#include "accuracy_test.h"
#include "test_trajectories.h"
#include "virtual_sensor.h"
#include "sensor_event.h"
#include "imu_sensor_model.h"
#include "mocap_sensor_model.h"
#include "gps_sensor_model.h"

namespace invariant::test
{
namespace
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

const auto test_trajectories = testing::Values(
    TestTrajectory{"StandStill 10s", stand_still(10)},
    TestTrajectory{"Rotate 3600 left, 10s", rotate_in_place(3600, 10)},
    TestTrajectory{"ConstantVel xy: 10m; 10s", constant_velocity({1,1,0}, 10)},
    TestTrajectory{"StartStop: {1,1,0}, 10s", start_stop({1,1,0}, 10)},
    TestTrajectory{"Circle: 360, 1m, 10s", circle(360, 1, 10)}
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
        VirtualSensor{ImuSensorModel{ImuNoiseLevel::Mueller18, 100.0}},
        VirtualSensor{GpsSensorModel{GpsNoiseLevel::None, 10.0}},
    },
    std::vector{
        VirtualSensor{ImuSensorModel{ImuNoiseLevel::Mueller18, 100.0}},
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
