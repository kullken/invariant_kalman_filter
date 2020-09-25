#include "mock_trajectory_node.h"

#include <string>

#include <ros/ros.h>

#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include <ugl_ros/convert_tf2.h>

#include <ugl/trajectory/trajectory.h>

#include "accuracy_test.h"
#include "sensor_models/imu_sensor_model.h"
#include "sensor_models/mocap_sensor_model.h"

namespace invariant::test
{

static constexpr double kDefaultGroundTruthFrequency = 100.0;

MockTrajectoryNode::MockTrajectoryNode(const ugl::trajectory::Trajectory &trajectory, const ImuSensorModel &imu, const MocapSensorModel &mocap)
    : m_trajectory(trajectory)
    , m_imu_model(imu)
    , m_mocap_model(mocap)
{
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");

    m_imu_pub   = nh.advertise<sensor_msgs::Imu>("imu", 10);
    m_mocap_pub = nh.advertise<geometry_msgs::PoseStamped>("mocap/pose", 10);

    m_odom_filtered_sub = nh.subscribe("odom_filtered", 10, &MockTrajectoryNode::odom_filtered_cb, this);

    m_base_frame = nh_private.param<std::string>("base_frame", "base_link");
    m_map_frame  = nh_private.param<std::string>("map_frame", "map");
    m_ground_truth_frame = nh_private.param<std::string>("ground_truth_frame", "base_link_ground_truth");

    constexpr bool oneshot = false;
    constexpr bool autostart = false;

    m_imu_timer = nh.createTimer(m_imu_model.period(), &MockTrajectoryNode::publish_imu, this, oneshot, autostart);
    m_mocap_timer = nh.createTimer(m_mocap_model.period(), &MockTrajectoryNode::publish_mocap, this, oneshot, autostart);

    const double ground_truth_frequency = nh_private.param<double>("ground_truth_frequency", kDefaultGroundTruthFrequency);
    m_ground_truth_timer = nh.createTimer(1.0/ground_truth_frequency, &MockTrajectoryNode::publish_ground_truth, this, oneshot, autostart);

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

void MockTrajectoryNode::stop()
{
    m_imu_timer.stop();
    m_mocap_timer.stop();
    m_ground_truth_timer.stop();
    // TODO: Stop state estimate subscriber.
    ROS_INFO("Node stoped!");
}

Result MockTrajectoryNode::get_result()
{
    const auto count = m_result.times.size();
    auto square_and_add = [](double sum, double item) { return sum + item*item; };

    m_result.position_rmse = std::sqrt(std::accumulate(std::cbegin(m_result.position_errors), std::cend(m_result.position_errors), 0.0, square_and_add) / count);
    m_result.velocity_rmse = std::sqrt(std::accumulate(std::cbegin(m_result.velocity_errors), std::cend(m_result.velocity_errors), 0.0, square_and_add) / count);
    m_result.rotation_rmse = std::sqrt(std::accumulate(std::cbegin(m_result.rotation_errors), std::cend(m_result.rotation_errors), 0.0, square_and_add) / count);

    return m_result;
}

void MockTrajectoryNode::publish_imu(const ros::TimerEvent& event)
{
    const ros::Time now = event.current_real;
    const double t = (now - m_t0).toSec();

    m_imu_msg.linear_acceleration = tf2::toMsg<geometry_msgs::Vector3>(m_imu_model.get_accel_reading(t, m_trajectory));
    m_imu_msg.angular_velocity = tf2::toMsg<geometry_msgs::Vector3>(m_imu_model.get_gyro_reading(t, m_trajectory));

    m_imu_msg.header.stamp = now;
    m_imu_pub.publish(m_imu_msg);
}

void MockTrajectoryNode::publish_mocap(const ros::TimerEvent& event)
{
    const ros::Time now = event.current_real;
    const double t = (now - m_t0).toSec();

    m_mocap_msg.pose = tf2::toMsg<geometry_msgs::Pose>(m_mocap_model.get_pose_reading(t, m_trajectory));

    m_mocap_msg.header.stamp = now;
    m_mocap_pub.publish(m_mocap_msg);
}

// TODO: Publish ground truth odometry information.
void MockTrajectoryNode::publish_ground_truth(const ros::TimerEvent& event)
{
    const ros::Time now = event.current_real;
    const double t = (now - m_t0).toSec();

    m_ground_truth_tf.transform.translation = tf2::toMsg<geometry_msgs::Vector3>(m_trajectory.get_position(t));
    m_ground_truth_tf.transform.rotation = tf2::toMsg(m_trajectory.get_quaternion(t));

    m_ground_truth_tf.header.stamp = now;
    m_tf_broadcaster.sendTransform(m_ground_truth_tf);
}

void MockTrajectoryNode::odom_filtered_cb(const nav_msgs::Odometry& msg)
{
    // TODO: Use 'ros::Time::now()' or 'msg.header.stamp' when calculating ground truth?
    const ros::Time now = ros::Time::now();
    const double t = (now - m_t0).toSec();

    // TODO: Make sure ground truth data and estimated data are in the same coordinate frame.
    const ugl::Vector3 true_pos = m_trajectory.get_position(t);
    const ugl::Vector3 true_vel = m_trajectory.get_velocity(t);
    const ugl::UnitQuaternion true_quat = m_trajectory.get_quaternion(t);

    const ugl::Vector3 predicted_pos = tf2::fromMsg(msg.pose.pose.position);
    const ugl::Vector3 predicted_vel = tf2::fromMsg(msg.twist.twist.linear);
    const ugl::UnitQuaternion predicted_quat = tf2::fromMsg(msg.pose.pose.orientation);

    const double pos_error = (true_pos - predicted_pos).norm();
    const double vel_error = (true_vel - predicted_vel).norm();
    const double rot_error = true_quat.angularDistance(predicted_quat);
    m_result.position_errors.push_back(pos_error);
    m_result.velocity_errors.push_back(vel_error);
    m_result.rotation_errors.push_back(rot_error);
    m_result.times.push_back(t);
}

} // namespace invariant::test
