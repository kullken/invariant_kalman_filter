#include "kalman_node.h"

#include <string>
#include <memory>

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <ugl/math/vector.h>
#include <ugl/math/matrix.h>

#include <ugl_ros/convert.h>

#include "measurement.h"

#include "iekf.h"
#include "iekf_types.h"

#include "ros_utils.h"

namespace invariant
{

static constexpr size_t k_queue_min_size = 5;
static constexpr size_t k_queue_max_size = 20;

KalmanNode::KalmanNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
    : m_nh(nh)
    , m_nh_private(nh_private)
    , m_base_frame(nh_private.param<std::string>("base_frame", "base_link"))
    , m_map_frame(nh_private.param<std::string>("map_frame", "map"))
{
    m_imu_sub       = m_nh.subscribe("imu", 10, &KalmanNode::imu_cb, this);
    m_mocap_sub     = m_nh.subscribe("mocap/pose", 10, &KalmanNode::mocap_cb, this);
    m_pose_pub      = m_nh.advertise<geometry_msgs::PoseStamped>("pose_filtered", 10);
    m_velocity_pub  = m_nh.advertise<geometry_msgs::Vector3Stamped>("vel_filtered", 10);

    const double frequency = m_nh_private.param<double>("frequency", 10.0);
    m_timer = m_nh.createTimer(1.0/frequency, &KalmanNode::timer_cb, this, false, false);

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
    ROS_INFO("Timer started!");
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

    const double roll  = m_nh_private.param<double>("init/roll" , 0.0);
    const double pitch = m_nh_private.param<double>("init/pitch", 0.0);
    const double yaw   = m_nh_private.param<double>("init/yaw"  , 0.0);
    // TODO: Lots of conceptually unneeded conversions. Make nicer somehow.
    tf2::Quaternion quat_tf;
    quat_tf.setRPY(roll, pitch, yaw);
    const ugl::Rotation R0 = tf2::fromMsg(tf2::toMsg(quat_tf));

    // TODO: Initilise from launch parameters?
    const Covariance<9> P0 = Covariance<9>::Identity() * 0.1;

    m_iekf_filter = IEKF(R0, p0, v0, P0);

    return;
}

void KalmanNode::timer_cb(const ros::TimerEvent& e)
{
    if (m_queue.size() > k_queue_max_size)
    {
        ROS_WARN("Measurement queue size [=%lu] exceeds max size [=%zu]! Filter might not be running in real-time.", m_queue.size(), k_queue_max_size);
    }

    while (m_queue.size() > k_queue_min_size)
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
    publish_pose(e.current_real);
    publish_velocity(e.current_real);
}

void KalmanNode::imu_cb(const sensor_msgs::Imu& msg)
{
    m_queue.push(std::make_shared<ImuMeasurement>(msg));
}
    
void KalmanNode::mocap_cb(const geometry_msgs::PoseStamped& msg)
{
    m_queue.push(std::make_shared<MocapMeasurement>(msg));
    if (m_queue.size() > k_queue_max_size)
    {
        ROS_WARN("Measurement queue exceeds max size [=%zu]! Filter might not be running in real-time.", k_queue_max_size);
    }
}

void KalmanNode::publish_tf(const ros::Time& stamp)
{
    geometry_msgs::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = m_map_frame;
    tf.child_frame_id = m_base_frame;
    tf2::toMsg(m_iekf_filter.get_pos(), tf.transform.translation);
    tf2::toMsg(m_iekf_filter.get_rot(), tf.transform.rotation);

    m_tf_broadcaster.sendTransform(tf);
}

void KalmanNode::publish_pose(const ros::Time& stamp)
{
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = m_map_frame;
    tf2::toMsg(m_iekf_filter.get_pos(), pose.pose.position);
    tf2::toMsg(m_iekf_filter.get_quat(), pose.pose.orientation);

    m_pose_pub.publish(pose);
}

void KalmanNode::publish_velocity(const ros::Time& stamp)
{
    geometry_msgs::Vector3Stamped vel;
    vel.header.stamp = stamp;
    vel.header.frame_id = m_map_frame;
    tf2::toMsg(m_iekf_filter.get_vel(), vel.vector);

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