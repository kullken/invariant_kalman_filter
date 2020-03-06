#pragma once

#include <string>

#include "ros/ros.h"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"

#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"

#include "iekf.h"
#include "iekf_types.h"

namespace iekf
{

class IEKFNode
{
private:
    ros::NodeHandle& m_nh;

    ros::Subscriber m_imu_sub;
    ros::Subscriber m_mocap_sub;

    ros::Publisher m_pose_pub;
    ros::Publisher m_velocity_pub;

    tf2_ros::Buffer m_tf_buff;
    tf2_ros::TransformListener m_tf_lstn;
    tf2_ros::TransformBroadcaster m_tf_br;

    ros::Timer m_timer;

    std::string m_base_frame;
    std::string m_odom_frame;
    std::string m_map_frame;

    IEKF m_iekf_filter;

    ros::Time m_previous_imu_time;

    bool m_imu_recieved;
    Vector3 m_imu_acc;
    Vector3 m_imu_ang_vel;

    bool m_mocap_recieved;
    Rotation m_mocap_R;
    Vector3 m_mocap_pos;

public:
    IEKFNode(ros::NodeHandle& nh);

    void start();

private:
    void initialise_iekf_filter();

    void timer_cb(const ros::TimerEvent& e);
    void imu_cb(const sensor_msgs::Imu& msg);
    void mocap_cb(const geometry_msgs::PoseStamped& msg);

    void publish_tf(const ros::Time& stamp);
    void publish_base_link_tf(const ros::Time& stamp);
    void publish_pose(const ros::Time& stamp);
    void publish_velocity(const ros::Time& stamp);
};

}