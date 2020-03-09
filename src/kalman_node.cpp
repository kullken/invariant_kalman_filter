#include "kalman_node.h"

#include <string>

#include <ros/ros.h>

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_eigen/tf2_eigen.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"

#include "iekf.h"
#include "iekf_types.h"
#include "iekf_tf2_conversions.h"

#include "ros_utils.h"

namespace invariant
{

constexpr size_t k_imu_buffer_capacity = 20;

KalmanNode::KalmanNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
    : m_nh(nh)
    , m_nh_private(nh_private)
    , m_base_frame( [&nh_private]() -> std::string { return nh_private.param<std::string>("base_frame", "base_link"); }() )
    , m_map_frame(  [&nh_private]() -> std::string { return nh_private.param<std::string>("map_frame", "map"); }() )
{
    m_imu_sub       = m_nh.subscribe("imu", 10, &KalmanNode::imu_cb, this);
    m_mocap_sub     = m_nh.subscribe("mocap/pose", 10, &KalmanNode::mocap_cb, this);
    m_pose_pub      = m_nh.advertise<geometry_msgs::PoseStamped>("pose_filtered", 10);
    m_velocity_pub  = m_nh.advertise<geometry_msgs::PointStamped>("vel_filtered", 10);

    const double frequency = m_nh_private.param<double>("frequency", 10.0);
    m_timer = m_nh.createTimer(1.0/frequency, &KalmanNode::timer_cb, this, false, false);

    m_imu_buffer.reserve(k_imu_buffer_capacity);

    ros_utils::wait_for_message<sensor_msgs::Imu>(m_imu_sub);

    // TODO: Measure accelerometer bias.
    initialise_iekf_filter();

    ROS_DEBUG_STREAM("State initialised to:" 
                    << "\nR = \n" << m_iekf_filter.get_rot()
                    << "\npos = \n" << m_iekf_filter.get_pos()
                    << "\nvel = \n" << m_iekf_filter.get_vel());
}

void KalmanNode::start()
{
    m_previous_imu_time = ros::Time::now();
    m_timer.start();
    ROS_INFO("Timer started!");
}

void KalmanNode::initialise_iekf_filter()
{
    // TODO: Initilise from launch parameters.
    const Vector3 p0 = Vector3::Zero();
    const Vector3 v0 = Vector3::Zero();
    const Rotation R0 = Rotation::Identity();
    const Covariance<9> P0 = Covariance<9>::Identity() * 0.1;

    m_iekf_filter = IEKF(R0, p0, v0, P0);

    return;
}

void KalmanNode::timer_cb(const ros::TimerEvent& e)
{
    for (const auto& imu_msg : m_imu_buffer)
    {
        const double dt = (imu_msg.header.stamp - m_previous_imu_time).toSec();
        if (dt > 0.05)
        {
            ROS_WARN("Time between IMU messages is high [dt=%f]. Might result in large discretisation errors.", dt);
        }
        m_iekf_filter.predict(dt, tf2::fromMsg(imu_msg.linear_acceleration), tf2::fromMsg(imu_msg.angular_velocity));
        m_previous_imu_time = imu_msg.header.stamp;
    }
    m_imu_buffer.clear();

    if (m_mocap_recieved)
    {
        m_iekf_filter.mocap_update(m_mocap_R, m_mocap_pos);
        m_mocap_recieved = false;
    }

    publish_tf(e.current_real);
    publish_pose(e.current_real);
    publish_velocity(e.current_real);
}

void KalmanNode::imu_cb(const sensor_msgs::Imu& msg)
{
    if (m_imu_buffer.size() < k_imu_buffer_capacity)
    {
        m_imu_buffer.push_back(msg);
    }
    else
    {
        ROS_ERROR("IMU buffer is full! Ignoring new IMU messages until buffer has room again.");
    }
}
    
void KalmanNode::mocap_cb(const geometry_msgs::PoseStamped& msg)
{
    tf2::fromMsg(msg.pose.orientation, m_mocap_R);
    tf2::fromMsg(msg.pose.position, m_mocap_pos);
    m_mocap_recieved = true;
}

void KalmanNode::publish_tf(const ros::Time& stamp)
{
    geometry_msgs::TransformStamped tf;

    tf.header.stamp     = stamp;
    tf.header.frame_id  = m_map_frame;
    tf.child_frame_id   = m_base_frame;

    tf2::toMsg(m_iekf_filter.get_pos(), tf.transform.translation);
    tf2::toMsg(m_iekf_filter.get_rot(), tf.transform.rotation);

    m_tf_broadcaster.sendTransform(tf);
}

void KalmanNode::publish_pose(const ros::Time& stamp)
{
    geometry_msgs::PoseStamped pose;

    pose.header.stamp     = stamp;
    pose.header.frame_id  = m_map_frame;
    pose.pose.position    = tf2::toMsg(m_iekf_filter.get_pos());
    pose.pose.orientation = tf2::toMsg(m_iekf_filter.get_quat());

    m_pose_pub.publish(pose);
}

void KalmanNode::publish_velocity(const ros::Time& stamp)
{
    geometry_msgs::PointStamped vel;

    vel.header.stamp    = stamp;
    vel.header.frame_id = m_map_frame;
    vel.point           = tf2::toMsg(m_iekf_filter.get_vel());

    m_velocity_pub.publish(vel);
}

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "kalman_node");
    ros::NodeHandle nh("");
    ros::NodeHandle nh_private("~");

    ROS_INFO("Initialising node...");
    invariant::KalmanNode node(nh, nh_private);
    ROS_INFO("Node initialisation done.");

    node.start();
    ros::spin();
}