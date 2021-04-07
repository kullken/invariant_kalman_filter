#ifndef INVARIANT_KALMAN_NODE_H
#define INVARIANT_KALMAN_NODE_H

#include <memory>
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
private:
    using MeasurementPtr = std::shared_ptr<const Measurement>;

    // Lower/earlier time stamp has higher priority.
    struct MeasurementPriority
    {
        bool operator()(const MeasurementPtr& lhs, const MeasurementPtr& rhs) const {
            return lhs->stamp() > rhs->stamp();
        }
    };

public:
    KalmanNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private);

    void start();

private:
    void initialise_iekf_filter();

    void imu_cb(const sensor_msgs::Imu& msg);
    void mocap_cb(const geometry_msgs::PoseStamped& msg);
    bool reset_cb(std_srvs::EmptyRequest&, std_srvs::EmptyResponse&);

    void timer_cb(const ros::TimerEvent& e);
    void process_imu_measurement(const ImuMeasurement& measurement);
    void process_mocap_measurement(const MocapMeasurement& measurement);

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
    MocapModel m_mocap_model;
    ImuModel m_imu_model;

    std::priority_queue<MeasurementPtr, std::vector<MeasurementPtr>, MeasurementPriority> m_queue;

    ros::Time m_previous_imu_time;
};

} // namespace invariant

#endif // INVARIANT_KALMAN_NODE_H
