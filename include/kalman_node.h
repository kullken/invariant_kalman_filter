#ifndef INVARIANT_KALMAN_NODE_H
#define INVARIANT_KALMAN_NODE_H

#include <string>
#include <vector>
#include <queue>

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_srvs/Empty.h>

#include "measurement.h"

#include "iekf.h"

namespace invariant
{

class KalmanNode
{
public:
    KalmanNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private);

    void start();

private:
    void initialise_iekf_filter();

    void imu_cb(const sensor_msgs::Imu& msg);
    void mocap_cb(const geometry_msgs::PoseStamped& msg);
    bool reset_cb(std_srvs::EmptyRequest&, std_srvs::EmptyResponse&);

    void timer_cb(const ros::TimerEvent& e);

    void publish_tf(const ros::Time& stamp);
    void publish_odometry(const ros::Time& stamp);

private:
    ros::NodeHandle& m_nh;
    ros::NodeHandle& m_nh_private;

    ros::Subscriber m_imu_sub;
    ros::Subscriber m_mocap_sub;

    ros::Publisher m_odom_pub;
    tf2_ros::TransformBroadcaster m_tf_broadcaster;

    ros::ServiceServer m_reset_server;

    geometry_msgs::TransformStamped m_tf_msg;
    nav_msgs::Odometry m_odom_msg;

    const std::string m_base_frame;
    const std::string m_map_frame;

    ros::Timer m_timer;

    IEKF m_iekf_filter;

    std::priority_queue<std::shared_ptr<Measurement>, std::vector<std::shared_ptr<Measurement>>, MeasurementCompare> m_queue;

    ros::Time m_previous_imu_time;
};

} // namespace invariant

#endif // INVARIANT_KALMAN_NODE_H
