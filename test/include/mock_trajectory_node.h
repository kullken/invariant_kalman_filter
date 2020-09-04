#ifndef INVARIANT_MOCK_TRAJECTORY_NODE_H
#define INVARIANT_MOCK_TRAJECTORY_NODE_H

#include <string>

#include <ros/ros.h>

#include <tf2_ros/transform_broadcaster.h>

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

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
    void publish_imu(const ros::TimerEvent& event);
    void publish_mocap(const ros::TimerEvent& event);
    void publish_ground_truth(const ros::TimerEvent& event);

private:
    ros::NodeHandle& m_nh;
    ros::NodeHandle& m_nh_private;

    ros::Publisher m_imu_pub;
    ros::Publisher m_mocap_pub;

    sensor_msgs::Imu m_imu_msg;
    geometry_msgs::PoseStamped m_mocap_msg;
    geometry_msgs::TransformStamped m_ground_truth_tf;

    tf2_ros::TransformBroadcaster m_tf_broadcaster;

    const std::string m_base_frame;
    const std::string m_map_frame;
    const std::string m_ground_truth_frame;

    ros::Time m_t0;
    ros::Timer m_imu_timer;
    ros::Timer m_mocap_timer;
    ros::Timer m_ground_truth_timer;

    ugl::trajectory::Trajectory m_trajectory;
    ImuSensorModel m_imu_model;
    MocapSensorModel m_mocap_model;
};

} // namespace invariant::test

#endif // INVARIANT_MOCK_TRAJECTORY_NODE_H
