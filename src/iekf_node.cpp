#include "iekf_node.h"

#include <string>

#include <ros/ros.h>

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_eigen/tf2_eigen.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"

#include "iekf.h"
#include "iekf_types.h"
#include "iekf_tf2_conversions.h"

#include "ros_utils.h"

namespace invariant
{

IEKFNode::IEKFNode(ros::NodeHandle& nh)
    : m_nh(nh)
    , m_imu_sub(m_nh.subscribe("imu", 10, &IEKFNode::imu_cb, this))
    , m_mocap_sub(m_nh.subscribe("mocap/pose", 10, &IEKFNode::mocap_cb, this))
    , m_pose_pub(m_nh.advertise<geometry_msgs::PoseStamped>("pose_filtered", 10))
    , m_velocity_pub(m_nh.advertise<geometry_msgs::PointStamped>("vel_filtered", 10))
    , m_tf_buff()
    , m_tf_lstn(m_tf_buff)
    , m_tf_br()
    , m_timer()
    , m_base_frame()
    , m_odom_frame()
    , m_map_frame()
    , m_iekf_filter()
    , m_previous_imu_time()
    , m_imu_recieved(false)
    , m_imu_acc(Vector3::Zero())
    , m_imu_ang_vel(Vector3::Zero())
    , m_mocap_recieved(false)
    , m_mocap_R(Rotation::Identity())
    , m_mocap_pos(Vector3::Zero())
{
    // Private nodehandle to read private parameters.
    ros::NodeHandle nh_private("~");

    // Parameters
    m_base_frame = nh_private.param<std::string>("base_frame", "base_link");
    m_odom_frame = nh_private.param<std::string>("odom_frame", "odom");
    m_map_frame  = nh_private.param<std::string>("map_frame", "map");
    const double frequency = nh_private.param("frequency", 10.0);

    m_timer = m_nh.createTimer(1.0/frequency, &IEKFNode::timer_cb, this, false, false);

    ros_utils::wait_for_message<sensor_msgs::Imu>(m_imu_sub);

    while (ros::ok())
    {
        try
        {
            m_tf_buff.lookupTransform(m_base_frame, m_odom_frame, ros::Time(0), ros::Duration(5));
            ROS_INFO("Transform %s -> %s found.", m_odom_frame.c_str(), m_base_frame.c_str());
            break;
        }
        catch(const tf2::LookupException&)
        {
            ROS_INFO("Waiting for transform %s -> %s...", m_odom_frame.c_str(), m_base_frame.c_str());
        }
    }
    // TODO: Measure accelerometer bias.
    initialise_iekf_filter();

    ROS_DEBUG_STREAM("State initialised to:" 
                    << "\nR = \n" << m_iekf_filter.get_rot()
                    << "\npos = \n" << m_iekf_filter.get_pos()
                    << "\nvel = \n" << m_iekf_filter.get_vel());
}

void IEKFNode::start()
{
    m_previous_imu_time = ros::Time::now();
    m_imu_recieved = false;
    m_mocap_recieved = false;
    m_timer.start();
    ROS_INFO("Timer started!");
}

void IEKFNode::initialise_iekf_filter()
{
    // TODO: Set initial values to something smarter.
    const Vector3 p0 = Vector3::Zero();
    const Vector3 v0 = Vector3::Zero();
    const Rotation R0 = Rotation::Identity();
    const Covariance<9> P0 = Covariance<9>::Identity() * 0.1;

    // m_iekf_filter = IEKF(R0, p0, v0, P0);

    // Temporary during testing!!!
    // Wait for first pose message published from recorded rosbag 
    // and use it to initialise the filter.
    // geometry_msgs::PoseStampedConstPtr pose;
    // do 
    // {
    //     pose = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("pose", m_nh);
    // // Skip messages like these. They exist at start of some rosbags. Unknown why, might be started recording to early.
    // // TODO: Rerecord rosbags so this can be removed.
    // } while (pose->pose.position.x == 0.0 && pose->pose.position.y == 0.0 && pose->pose.position.z == 0.0
    //          && pose->pose.orientation.x == 1.0 && pose->pose.orientation.y == 0.0 
    //          && pose->pose.orientation.z == 0.0&& pose->pose.orientation.w == 0.0); 

    // Vector3 p0;
    // tf2::fromMsg(pose->pose.position, p0);
    // Quaternion q0;
    // tf2::fromMsg(pose->pose.orientation, q0);
    // Rotation R0 = q0.toRotationMatrix();

    // ROS_DEBUG_STREAM("Initial quaternion:\n" << q0.x() << " " << q0.y() << " " << q0.z() << " " << q0.w() << "\n");
    // ROS_DEBUG_STREAM("Initial rotation matrix:\n" << R0);

    // const Vector3 v0 = Vector3::Zero();
    // const Covariance<9> P0 = Covariance<9>::Identity() * 0.1;

    m_iekf_filter = IEKF(R0, p0, v0, P0);

    return;
}

void IEKFNode::timer_cb(const ros::TimerEvent& e)
{
    if (m_imu_recieved)
    {
        const double dt = (e.current_real - m_previous_imu_time).toSec();
        m_iekf_filter.predict(dt, m_imu_acc, m_imu_ang_vel);
        m_previous_imu_time = e.current_real;
        m_imu_recieved = false;
    }
    if (m_mocap_recieved)
    {
        m_iekf_filter.mocap_update(m_mocap_R, m_mocap_pos);
        m_mocap_recieved = false;
    }

    publish_tf(e.current_real);
    publish_base_link_tf(e.current_real);   // Only for debugging.
    publish_pose(e.current_real);
    publish_velocity(e.current_real);
}

void IEKFNode::imu_cb(const sensor_msgs::Imu& msg)
{
    tf2::fromMsg(msg.linear_acceleration, m_imu_acc);
    tf2::fromMsg(msg.angular_velocity, m_imu_ang_vel);
    m_imu_recieved = true;
}
    
void IEKFNode::mocap_cb(const geometry_msgs::PoseStamped& msg)
{
    tf2::fromMsg(msg.pose.orientation, m_mocap_R);
    tf2::fromMsg(msg.pose.position, m_mocap_pos);
    m_mocap_recieved = true;
}

void IEKFNode::publish_tf(const ros::Time& stamp)
{
    geometry_msgs::TransformStamped tf_odom_base = m_tf_buff.lookupTransform(m_base_frame, m_odom_frame, ros::Time(0));
    Matrix<4,4> T_base_odom = Matrix<4,4>::Identity();
    {
        Rotation R;
        Vector3 pos;
        tf2::fromMsg(tf_odom_base.transform.rotation, R);
        tf2::fromMsg(tf_odom_base.transform.translation, pos);
        T_base_odom.block<3,3>(0,0) = R;
        T_base_odom.block<3,1>(0,3) = pos;
    }

    Matrix<4,4> T_map_base = Matrix<4,4>::Identity();
    T_map_base.block<3,3>(0,0) = m_iekf_filter.get_rot();
    T_map_base.block<3,1>(0,3) = m_iekf_filter.get_pos();


    Matrix<4,4> T_map_odom = T_map_base * T_base_odom;

    // Publish partial transform map->odom for usage by other systems.
    geometry_msgs::TransformStamped tf;

    tf.header.stamp     = stamp;
    tf.header.frame_id  = m_map_frame;
    tf.child_frame_id   = m_odom_frame;

    {
        Rotation R = T_map_odom.block<3,3>(0,0);
        Vector3 pos = T_map_odom.block<3,1>(0,3);
        tf2::toMsg(R, tf.transform.rotation);
        tf2::toMsg(pos, tf.transform.translation);
    }

    m_tf_br.sendTransform(tf);
}

void IEKFNode::publish_base_link_tf(const ros::Time& stamp)
{
    // Publish complete transform map->base_link for debugging.
    geometry_msgs::TransformStamped tf;

    tf.header.stamp     = stamp;
    tf.header.frame_id  = m_map_frame;
    tf.child_frame_id   = m_base_frame + "_iekf";

    tf2::toMsg(m_iekf_filter.get_rot(), tf.transform.rotation);
    tf2::toMsg(m_iekf_filter.get_pos(), tf.transform.translation);

    m_tf_br.sendTransform(tf);
}

void IEKFNode::publish_pose(const ros::Time& stamp)
{
    geometry_msgs::PoseStamped pose;

    pose.header.stamp     = stamp;
    pose.header.frame_id  = m_odom_frame;
    pose.pose.position    = tf2::toMsg(m_iekf_filter.get_pos());
    pose.pose.orientation = tf2::toMsg(m_iekf_filter.get_quat());

    m_pose_pub.publish(pose);
}

void IEKFNode::publish_velocity(const ros::Time& stamp)
{
    geometry_msgs::PointStamped vel;

    vel.header.stamp    = stamp;
    vel.header.frame_id = m_odom_frame;
    vel.point           = tf2::toMsg(m_iekf_filter.get_vel());

    m_velocity_pub.publish(vel);
}

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ekf_node");
    ros::NodeHandle nh;

    ROS_INFO("Initialising node...");
    invariant::IEKFNode node(nh);
    ROS_INFO("Node initialisation done.");

    node.start();
    ros::spin();
}