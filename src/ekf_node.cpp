#include "ekf_node.h"

#include <string>

#include <ros/ros.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include "ekf.h"
#include "ekf_types.h"

namespace ekf
{

EKFNode::EKFNode(ros::NodeHandle& nh)
    : m_nh(nh)
    , m_imu_sub(m_nh.subscribe("imu", 10, &EKFNode::imu_cb, this))
    , m_pose_pub(m_nh.advertise<geometry_msgs::PoseStamped>("pose_filtered", 10))
    , m_velocity_pub(m_nh.advertise<geometry_msgs::PointStamped>("vel_filtered", 10))
    , m_tf_buff()
    , m_tf_lstn(m_tf_buff)
    , m_tf_br()
    , m_timer()
    , m_timer_period()
    , m_base_frame()
    , m_odom_frame()
    , m_ekf_filter()
    , m_measured_acc(Vec3::Zero())
    , m_measured_ang_vel(Vec3::Zero())
    , m_measured_pos(Vec3::Zero())
{
    // Private nodehandle to read private parameters.
    ros::NodeHandle nh_private("~");

    // Parameters
    nh_private.param<std::string>("base_frame", m_base_frame, "base_link");
    nh_private.param<std::string>("odom_frame", m_odom_frame, "odom");

    double frequency;
    nh_private.param<double>("frequency", frequency, 10);

    m_timer_period = ros::Duration(ros::Rate(frequency));
    m_timer = m_nh.createTimer(m_timer_period, &EKFNode::timer_cb, this, false, false);

    ROS_INFO("Waiting for imu message...");
    ros::topic::waitForMessage<sensor_msgs::Imu>("imu", m_nh);
    ROS_INFO("Imu message recieved.");

    // TODO: Measure accelerometer bias.
    m_ekf_filter = initialise_ekf_filter();

    ROS_INFO_STREAM("State initialised to:\n" 
                    << "\nR = \n" << m_ekf_filter.get_rot()
                    << "\npos = \n" << m_ekf_filter.get_pos()
                    << "\nvel = \n" << m_ekf_filter.get_vel());
}

void EKFNode::start()
{
    m_timer.start();
    ROS_INFO("Timer started!");
}

EKF EKFNode::initialise_ekf_filter() const
{
    // // TODO: Set initial values to something smarter.
    // const Vec3 p0 = Vec3::Zero();
    // const Vec3 v0 = Vec3::Zero();
    // const Rotation R0 = Rotation::Identity();
    // const Covariance P0 = Covariance::Identity() * 0.1;

    // Temporary during testing!!!
    // Wait for first pose message published from recorded rosbag 
    // and use it to initialise the filter.
    geometry_msgs::PoseStampedConstPtr pose;
    do 
    {
        pose = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("pose", m_nh);
    // Skip messages like these. They exist at start of some rosbags. Unknown why, might be started recording to early.
    // TODO: Rerecord rosbags so this can be removed.
    } while (pose->pose.position.x == 0.0 && pose->pose.position.y == 0.0 && pose->pose.position.z == 0.0
             && pose->pose.orientation.x == 1.0 && pose->pose.orientation.y == 0.0 
             && pose->pose.orientation.z == 0.0&& pose->pose.orientation.w == 0.0); 

    Vec3 p0;
    tf2::fromMsg(pose->pose.position, p0);
    Quaternion q0;
    tf2::fromMsg(pose->pose.orientation, q0);
    Rotation R0 = q0.toRotationMatrix();

    ROS_DEBUG_STREAM("Initial quaternion:\n" << q0.x() << " " << q0.y() << " " << q0.z() << " " << q0.w() << "\n");
    ROS_DEBUG_STREAM("Initial rotation matrix:\n" << R0);

    const Vec3 v0 = Vec3::Zero();
    const Covariance P0 = Covariance::Identity() * 0.1;

    return EKF(p0, v0, R0, P0);
}

void EKFNode::timer_cb(const ros::TimerEvent& e)
{
    ROS_DEBUG("Timer callback called!");
    if (e.last_real == ros::Time(0))
    {
        ROS_DEBUG("First call to timer. Skipping.");
        return;
    }

    const double dt = (e.current_real - e.last_real).toSec();
    filter_step(dt);

    publish_tf(e.current_real);
    publish_pose(e.current_real);
    publish_velocity(e.current_real);
}

void EKFNode::filter_step(double dt)
{
    m_ekf_filter.predict(dt, m_measured_acc, m_measured_ang_vel);
    // m_ekf_filter.update_with_position(m_measured_pos); // Note: There is no subscriber to pose data so m_measured_pose is always (0,0,0).
}

void EKFNode::imu_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
    tf2::fromMsg(msg->linear_acceleration, m_measured_acc);
    tf2::fromMsg(msg->angular_velocity, m_measured_ang_vel);
}

void EKFNode::publish_tf(const ros::Time& stamp)
{
    geometry_msgs::TransformStamped tf;

    tf.header.stamp     = stamp;
    tf.header.frame_id  = m_odom_frame;
    tf.child_frame_id   = m_base_frame;

    tf2::toMsg(m_ekf_filter.get_pos(), tf.transform.translation);
    tf.transform.rotation = tf2::toMsg(m_ekf_filter.get_quat());

    m_tf_br.sendTransform(tf);
}

void EKFNode::publish_pose(const ros::Time& stamp)
{
    geometry_msgs::PoseStamped pose;

    pose.header.stamp     = stamp;
    pose.header.frame_id  = m_odom_frame;
    pose.pose.position    = tf2::toMsg(m_ekf_filter.get_pos());
    pose.pose.orientation = tf2::toMsg(m_ekf_filter.get_quat());

    m_pose_pub.publish(pose);
}

void EKFNode::publish_velocity(const ros::Time& stamp)
{
    geometry_msgs::PointStamped vel;

    vel.header.stamp    = stamp;
    vel.header.frame_id = m_odom_frame;
    vel.point           = tf2::toMsg(m_ekf_filter.get_vel());

    m_velocity_pub.publish(vel);
}

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ekf_node");
    ros::NodeHandle nh;

    ROS_INFO("Initialising node...");
    ekf::EKFNode node(nh);
    ROS_INFO("Node initialisation done.");

    node.start();
    ros::spin();
}