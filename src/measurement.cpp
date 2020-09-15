#include "measurement.h"

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>

#include <ugl_ros/convert_tf2.h>

namespace invariant
{

ImuMeasurement::ImuMeasurement(const sensor_msgs::Imu& imu_msg)
    : Measurement(imu_msg.header.stamp)
    , m_acc(tf2::fromMsg(imu_msg.linear_acceleration))
    , m_rate(tf2::fromMsg(imu_msg.angular_velocity))
{
}

MocapMeasurement::MocapMeasurement(const geometry_msgs::PoseStamped& pose_msg)
    : Measurement(pose_msg.header.stamp)
    , m_pose(tf2::fromMsg(pose_msg.pose.orientation), tf2::fromMsg(pose_msg.pose.position))
{
}

} // namespace invariant
