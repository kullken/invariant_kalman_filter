#pragma once

#include <string>

#include <ros/ros.h>

#include <tf2_ros/transform_broadcaster.h>

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>

#include <ugl/trajectory/trajectory.h>

#include "sensor_models/imu_sensor_model.h"
#include "sensor_models/mocap_sensor_model.h"

namespace invariant::test
{

class MockTrajectoryNode
{
public:
    MockTrajectoryNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private);

    void start();

private:
    void publish_imu(const ros::TimerEvent& e);
    void publish_mocap(const ros::TimerEvent& e);

private:
    ros::NodeHandle& m_nh;
    ros::NodeHandle& m_nh_private;

    ros::Publisher m_imu_pub;
    ros::Publisher m_mocap_pub;

    sensor_msgs::Imu m_imu_msg;
    geometry_msgs::PoseStamped m_mocap_msg;

    tf2_ros::TransformBroadcaster m_tf_broadcaster;

    const std::string m_base_frame;
    const std::string m_map_frame;

    ros::Time m_t0;
    ros::Timer m_imu_timer;
    ros::Timer m_mocap_timer;

    const ugl::trajectory::Trajectory m_trajectory;
    ImuSensorModel m_imu_model;
    MocapSensorModel m_mocap_model;
};

}