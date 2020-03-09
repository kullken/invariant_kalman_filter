#pragma once

#include <string>

#include <ros/ros.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <sensor_msgs/Imu.h>

#include "ekf.h"
#include "ekf_types.h"

namespace ekf
{

class EKFNode
{
private:
    ros::NodeHandle& m_nh;

    ros::Subscriber m_imu_sub;
    ros::Publisher m_pose_pub;
    ros::Publisher m_velocity_pub;

    tf2_ros::Buffer m_tf_buff;
    tf2_ros::TransformListener m_tf_lstn;
    tf2_ros::TransformBroadcaster m_tf_br;

    ros::Timer m_timer;
    ros::Duration m_timer_period;

    std::string m_base_frame;
    std::string m_odom_frame;

    EKF m_ekf_filter;

    Vec3 m_measured_acc;
    Vec3 m_measured_ang_vel;
    Vec3 m_measured_pos;

public:
    EKFNode(ros::NodeHandle& nh);

    void start();

private:
    EKF initialise_ekf_filter() const;
    void filter_step(double dt);

    void timer_cb(const ros::TimerEvent& e);
    void imu_cb(const sensor_msgs::Imu::ConstPtr& msg);

    void publish_tf(const ros::Time& stamp);
    void publish_pose(const ros::Time& stamp);
    void publish_velocity(const ros::Time& stamp);
};

}