#include "kalman_node.h"

#include <string>
#include <memory>

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include <ugl/math/vector.h>
#include <ugl/math/matrix.h>
#include <ugl/math/quaternion.h>
#include <ugl/lie_group/rotation.h>

#include <ugl_ros/convert_tf2.h>

#include "measurement.h"

#include "iekf.h"

#include "ros_utils.h"

namespace invariant
{

static constexpr size_t kQueueMinSize = 5;
static constexpr size_t kQueueMaxSize = 20;

KalmanNode::KalmanNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
    : m_nh(nh)
    , m_nh_private(nh_private)
    , m_reset_server(nh_private.advertiseService("reset", &KalmanNode::reset_cb, this))
    , m_base_frame(nh_private.param<std::string>("base_frame", "base_link"))
    , m_map_frame(nh_private.param<std::string>("map_frame", "map"))
{
    m_imu_sub       = m_nh.subscribe("imu", 10, &KalmanNode::imu_cb, this);
    m_mocap_sub     = m_nh.subscribe("mocap/pose", 10, &KalmanNode::mocap_cb, this);
    m_odom_pub      = m_nh_private.advertise<nav_msgs::Odometry>("odometry", 10);

    const double frequency = m_nh_private.param<double>("frequency", 10.0);
    m_timer = m_nh.createTimer(1.0/frequency, &KalmanNode::timer_cb, this, false, false);

    m_tf_msg.header.frame_id = m_map_frame;
    m_tf_msg.child_frame_id = m_base_frame;
    m_odom_msg.header.frame_id = m_map_frame;
    m_odom_msg.child_frame_id = m_base_frame;

    initialise_iekf_filter();

    // TODO: Measure accelerometer bias.

    ros_utils::wait_for_message<sensor_msgs::Imu>(m_imu_sub);

    ROS_DEBUG_STREAM("State initialised to:"
                    << "\nR = \n" << m_iekf_filter.get_rot()
                    << "\npos = \n" << m_iekf_filter.get_pos()
                    << "\nvel = \n" << m_iekf_filter.get_vel());
}

void KalmanNode::start()
{
    m_previous_imu_time = ros::Time::now();
    m_timer.start();
}

void KalmanNode::initialise_iekf_filter()
{
    const double x0 = m_nh_private.param<double>("init/x", 0.0);
    const double y0 = m_nh_private.param<double>("init/y", 0.0);
    const double z0 = m_nh_private.param<double>("init/z", 0.0);
    const ugl::Vector3 p0{x0, y0, z0};

    const double vx0 = m_nh_private.param<double>("init/vx", 0.0);
    const double vy0 = m_nh_private.param<double>("init/vy", 0.0);
    const double vz0 = m_nh_private.param<double>("init/vz", 0.0);
    const ugl::Vector3 v0{vx0, vy0, vz0};

    const double qx = m_nh_private.param<double>("init/qx", 0.0);
    const double qy = m_nh_private.param<double>("init/qy", 0.0);
    const double qz = m_nh_private.param<double>("init/qz", 0.0);
    const double qw = m_nh_private.param<double>("init/qw", 0.0);
    const ugl::UnitQuaternion q0{qw, qx, qy, qz};
    const ugl::lie::Rotation R0{q0};

    // TODO: Initilise from launch parameters?
    const IEKF::Covariance<9> P0 = IEKF::Covariance<9>::Identity() * 0.1;

    m_iekf_filter = IEKF(R0, p0, v0, P0);

    return;
}

void KalmanNode::imu_cb(const sensor_msgs::Imu& msg)
{
    m_queue.push(std::make_shared<ImuMeasurement>(msg));
}

void KalmanNode::mocap_cb(const geometry_msgs::PoseStamped& msg)
{
    m_queue.push(std::make_shared<MocapMeasurement>(msg));
}

bool KalmanNode::reset_cb(std_srvs::EmptyRequest&, std_srvs::EmptyResponse&)
{
    ROS_INFO("Resetting...");
    m_timer.stop();
    initialise_iekf_filter();
    while (!m_queue.empty()) {
        m_queue.pop();
    }
    start();
    ROS_INFO("Reset done.");
    return true;
}

void KalmanNode::timer_cb(const ros::TimerEvent& e)
{
    if (m_queue.size() > kQueueMaxSize)
    {
        ROS_WARN("Measurement queue size [=%lu] exceeds max size [=%zu]! Filter might not be running in real-time.", m_queue.size(), kQueueMaxSize);
    }

    while (m_queue.size() > kQueueMinSize)
    {
        auto measurement_ptr = m_queue.top();
        m_queue.pop();

        switch (measurement_ptr->get_type())
        {
            case MeasurementType::imu:
            {
                auto imu_measurement_ptr = std::dynamic_pointer_cast<ImuMeasurement>(measurement_ptr);
                const sensor_msgs::Imu& imu_msg = imu_measurement_ptr->get_data();
                const double dt = (imu_msg.header.stamp - m_previous_imu_time).toSec();
                if (dt > 0.05)
                {
                    ROS_WARN("Time between IMU messages is high [dt=%f]. Might result in large discretisation errors.", dt);
                }
                m_iekf_filter.predict(dt, tf2::fromMsg(imu_msg.linear_acceleration), tf2::fromMsg(imu_msg.angular_velocity));
                m_previous_imu_time = imu_msg.header.stamp;
                break;
            }
            case MeasurementType::mocap:
            {
                auto mocap_measurement_ptr = std::dynamic_pointer_cast<MocapMeasurement>(measurement_ptr);
                const geometry_msgs::PoseStamped& mocap_msg = mocap_measurement_ptr->get_data();
                m_iekf_filter.mocap_update(tf2::fromMsg(mocap_msg.pose.orientation), tf2::fromMsg(mocap_msg.pose.position));
                break;
            }
        }
    }

    publish_tf(e.current_real);
    publish_odometry(e.current_real);
}

void KalmanNode::publish_tf(const ros::Time& stamp)
{
    m_tf_msg.header.stamp = stamp;
    tf2::toMsg(m_iekf_filter.get_pos(), m_tf_msg.transform.translation);
    tf2::toMsg(m_iekf_filter.get_rot(), m_tf_msg.transform.rotation);
    m_tf_broadcaster.sendTransform(m_tf_msg);
}

void KalmanNode::publish_odometry(const ros::Time& stamp)
{
    // TODO: Set covariance values in odometry message.
    m_odom_msg.header.stamp = stamp;
    tf2::toMsg(m_iekf_filter.get_pos(), m_odom_msg.pose.pose.position);
    tf2::toMsg(m_iekf_filter.get_quat(), m_odom_msg.pose.pose.orientation);
    tf2::toMsg(m_iekf_filter.get_vel(), m_odom_msg.twist.twist.linear);
    m_odom_pub.publish(m_odom_msg);
}

} // namespace invariant

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