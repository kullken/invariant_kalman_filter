#include "rosbag_helpers.h"

#include <algorithm>
#include <string>
#include <vector>

#include <ros/time.h>
#include <rosbag/bag.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>

#include <ugl/lie_group/rotation.h>
#include <ugl/lie_group/extended_pose.h>
#include <ugl_ros/convert_tf2.h>

#include "accuracy_test.h"
#include "sensor_event.h"


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
        const std::vector<ugl::Vector3>& errors,
        const std::vector<double>& times,
        const std::string& topic)
{
    const auto size = times.size();
    for (std::size_t i = 0; i < size; ++i)
    {
        const ros::Time stamp = to_rosbag_time(times[i]);
        std_msgs::Float64 error{};
        error.data = errors[i].norm();
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

void write_sensor_event(rosbag::Bag& rosbag, const ImuData& data, double time, const std::string& topic_prefix)
{
    sensor_msgs::Imu msg;
    msg.header.stamp = to_rosbag_time(time);
    msg.header.frame_id = "base_link";
    tf2::toMsg(data.acc, msg.linear_acceleration);
    tf2::toMsg(data.rate, msg.angular_velocity);
    rosbag.write(topic_prefix + "/imu", msg.header.stamp, msg);
}

void write_sensor_event(rosbag::Bag& rosbag, const GpsData& data, double time, const std::string& topic_prefix)
{
    geometry_msgs::PointStamped point;
    point.header.stamp = to_rosbag_time(time);
    point.header.frame_id = "map";
    tf2::toMsg(data.measurement.vector(), point.point);
    rosbag.write(topic_prefix + "/gps", point.header.stamp, point);
}

void write_sensor_event(rosbag::Bag& rosbag, const MocapData& data, double time, const std::string& topic_prefix)
{
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = to_rosbag_time(time);
    pose.header.frame_id = "map";
    tf2::toMsg(data.measurement, pose.pose);
    rosbag.write(topic_prefix + "/mocap", pose.header.stamp, pose);
}

void write_events(rosbag::Bag& rosbag, const std::vector<SensorEvent>& events, const std::string& topic_prefix)
{
    for (const auto& event: events)
    {
        const auto time = event.time();
        const auto visitor = [&](auto&& data) {
            write_sensor_event(rosbag, data, time, topic_prefix);
        };
        std::visit(visitor, event.get_variant());
    }
}

} // namespace

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

    write_events(rosbag, events, "/sensors");
}

} // namespace invariant::test
