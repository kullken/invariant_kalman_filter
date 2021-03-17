#include "sensor_event.h"

#include <string>
#include <variant>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>

#include <ugl_ros/convert_tf2.h>

namespace invariant::test
{
namespace
{

ros::Time to_rosbag_time(double t)
{
    // For some reason it is not allowed to add a timestamp of exactly 0 to a rosbag.
    return t == 0.0 ? ros::TIME_MIN : ros::Time{t};
}

void write_data(rosbag::Bag& rosbag, const ImuData& data, double time, const std::string& topic_prefix)
{
    sensor_msgs::Imu msg;
    msg.header.stamp = to_rosbag_time(time);
    msg.header.frame_id = "base_link";
    tf2::toMsg(data.acc, msg.linear_acceleration);
    tf2::toMsg(data.rate, msg.angular_velocity);
    rosbag.write(topic_prefix + "/imu", msg.header.stamp, msg);
}

void write_data(rosbag::Bag& rosbag, const GpsData& data, double time, const std::string& topic_prefix)
{
    geometry_msgs::PointStamped point;
    point.header.stamp = to_rosbag_time(time);
    point.header.frame_id = "map";
    tf2::toMsg(data.measurement.vector(), point.point);
    rosbag.write(topic_prefix + "/gps", point.header.stamp, point);
}

void write_data(rosbag::Bag& rosbag, const MocapData& data, double time, const std::string& topic_prefix)
{
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = to_rosbag_time(time);
    pose.header.frame_id = "map";
    tf2::toMsg(data.measurement, pose.pose);
    rosbag.write(topic_prefix + "/mocap", pose.header.stamp, pose);
}

} // namespace

void SensorEvent::write_to_rosbag(rosbag::Bag& rosbag, const std::string& topic_prefix) const
{
    auto visitor = [&](auto&& data) {
        write_data(rosbag, data, m_time, topic_prefix);
    };
    std::visit(visitor, m_data);
}

} // namespace invariant::test
