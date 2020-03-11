#pragma once

#include <string>
#include <vector>
#include <queue>

#include <ros/ros.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>

#include "measurement.h"

#include "iekf.h"
#include "iekf_types.h"

namespace invariant
{

class KalmanNode
{
private:
    ros::NodeHandle& m_nh;
    ros::NodeHandle& m_nh_private;

    ros::Subscriber m_imu_sub;
    ros::Subscriber m_mocap_sub;

    ros::Publisher m_pose_pub;
    ros::Publisher m_velocity_pub;

    tf2_ros::TransformBroadcaster m_tf_broadcaster;

    const std::string m_base_frame;
    const std::string m_map_frame;

    ros::Timer m_timer;

    IEKF m_iekf_filter;

    std::priority_queue<std::shared_ptr<Measurement>, std::vector<std::shared_ptr<Measurement>>, MeasurementCompare> m_queue;

    ros::Time m_previous_imu_time;

public:
    KalmanNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private);

    void start();

private:
    void initialise_iekf_filter();

    void timer_cb(const ros::TimerEvent& e);
    void imu_cb(const sensor_msgs::Imu& msg);
    void mocap_cb(const geometry_msgs::PoseStamped& msg);

    void publish_tf(const ros::Time& stamp);
    void publish_pose(const ros::Time& stamp);
    void publish_velocity(const ros::Time& stamp);
};

}