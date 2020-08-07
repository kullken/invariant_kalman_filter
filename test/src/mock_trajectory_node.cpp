#include "mock_trajectory_node.h"

#include <string>

#include <ros/ros.h>

#include <tf2_ros/transform_broadcaster.h>

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <ugl_ros/convert_tf2.h>

namespace invariant::test
{

static constexpr double kDefaultGroundTruthFrequency = 100.0;

MockTrajectoryNode::MockTrajectoryNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
    : m_nh(nh)
    , m_nh_private(nh_private)
    , m_base_frame(nh_private.param<std::string>("base_frame", "base_link"))
    , m_map_frame(nh_private.param<std::string>("map_frame", "map"))
    , m_ground_truth_frame(nh_private.param<std::string>("ground_truth_frame", "base_link_ground_truth"))
{
    m_imu_pub   = m_nh.advertise<sensor_msgs::Imu>("imu", 10);
    m_mocap_pub = m_nh.advertise<geometry_msgs::PoseStamped>("mocap/pose", 10);

    constexpr bool oneshot = false;
    constexpr bool autostart = false;

    m_imu_timer = m_nh.createTimer(m_imu_model.period(), &MockTrajectoryNode::publish_imu, this, oneshot, autostart);
    m_mocap_timer = m_nh.createTimer(m_mocap_model.period(), &MockTrajectoryNode::publish_mocap, this, oneshot, autostart);

    const double ground_truth_frequency = m_nh_private.param<double>("ground_truth_frequency", kDefaultGroundTruthFrequency);
    m_ground_truth_timer = m_nh.createTimer(1.0/ground_truth_frequency, &MockTrajectoryNode::publish_ground_truth, this, oneshot, autostart);

    m_imu_msg.header.frame_id = m_base_frame;
    m_mocap_msg.header.frame_id = m_map_frame;
    m_ground_truth_tf.header.frame_id = m_map_frame;
    m_ground_truth_tf.child_frame_id = m_ground_truth_frame;
}

void MockTrajectoryNode::start()
{
    m_t0 = ros::Time::now();
    m_imu_timer.start();
    m_mocap_timer.start();
    m_ground_truth_timer.start();
    ROS_INFO("Node started!");
}

void MockTrajectoryNode::publish_imu(const ros::TimerEvent&)
{
    const ros::Time now = ros::Time::now();
    const double t = (now - m_t0).toSec();

    m_imu_msg.linear_acceleration = tf2::toMsg<geometry_msgs::Vector3>(m_imu_model.get_accel_reading(t));
    m_imu_msg.angular_velocity = tf2::toMsg<geometry_msgs::Vector3>(m_imu_model.get_gyro_reading(t));

    m_imu_msg.header.stamp = now;
    m_imu_pub.publish(m_imu_msg);
}

void MockTrajectoryNode::publish_mocap(const ros::TimerEvent&)
{
    const ros::Time now = ros::Time::now();
    const double t = (now - m_t0).toSec();

    m_mocap_msg.pose.position = tf2::toMsg<geometry_msgs::Point>(m_mocap_model.get_pos_reading(t));
    m_mocap_msg.pose.orientation = tf2::toMsg(m_mocap_model.get_quat_reading(t));

    m_mocap_msg.header.stamp = now;
    m_mocap_pub.publish(m_mocap_msg);
}

// TODO: Publish ground truth odometry information.
void MockTrajectoryNode::publish_ground_truth(const ros::TimerEvent&)
{
    const ros::Time now = ros::Time::now();
    const double t = (now - m_t0).toSec();

    m_ground_truth_tf.transform.translation = tf2::toMsg<geometry_msgs::Vector3>(m_trajectory.get_position(t));
    m_ground_truth_tf.transform.rotation = tf2::toMsg(m_trajectory.get_quaternion(t));

    m_ground_truth_tf.header.stamp = now;
    m_tf_broadcaster.sendTransform(m_ground_truth_tf);
}

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mock_trajectory_node");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");

    ROS_INFO("Initialising node...");
    invariant::test::MockTrajectoryNode node(nh, nh_private);
    ROS_INFO("Node initialisation done.");

    node.start();
    ros::spin();
}