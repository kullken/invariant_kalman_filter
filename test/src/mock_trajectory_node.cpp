#include "mock_trajectory_node.h"

#include <string>

#include <ros/ros.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <tf2/LinearMath/Quaternion.h>

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

namespace invariant::test
{

static constexpr double kDefaultImuFrequency     = 100.0;
static constexpr double kDefaultMocapFrequency   = 100.0;

MockTrajectoryNode::MockTrajectoryNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
    : m_nh(nh)
    , m_nh_private(nh_private)
    , m_base_frame( [&nh_private]() -> std::string { return nh_private.param<std::string>("base_frame", "base_link"); }() )
    , m_map_frame(  [&nh_private]() -> std::string { return nh_private.param<std::string>("map_frame", "map"); }() )
{
    m_imu_pub   = m_nh.advertise<sensor_msgs::Imu>("imu", 10);
    m_mocap_pub = m_nh.advertise<geometry_msgs::PoseStamped>("mocap/pose", 10);

    const double imu_frequency = m_nh_private.param<double>("imu_frequency", kDefaultImuFrequency);
    m_imu_timer = m_nh.createTimer(1.0/imu_frequency, &MockTrajectoryNode::publish_imu, this, false, false);

    const double mocap_frequency = m_nh_private.param<double>("mocap_frequency", kDefaultMocapFrequency);
    m_mocap_timer = m_nh.createTimer(1.0/mocap_frequency, &MockTrajectoryNode::publish_mocap, this, false, false);
}

void MockTrajectoryNode::start()
{
    m_imu_timer.start();
    m_mocap_timer.start();
    ROS_INFO("Timer started!");
}

void MockTrajectoryNode::publish_imu(const ros::TimerEvent&)
{
    sensor_msgs::Imu imu;

    // Write stuff to imu message

    m_imu_pub.publish(imu);
}

void MockTrajectoryNode::publish_mocap(const ros::TimerEvent&)
{
    geometry_msgs::PoseStamped mocap;

    // Write stuff to mocap message

    m_mocap_pub.publish(mocap);
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