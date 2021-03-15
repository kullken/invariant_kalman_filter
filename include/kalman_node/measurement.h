#ifndef INVARIANT_MEASUREMENT_H
#define INVARIANT_MEASUREMENT_H

#include <ros/time.h>

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>

#include <ugl/math/vector.h>
#include <ugl/lie_group/pose.h>

namespace invariant
{

class Measurement
{
public:
    virtual ~Measurement() = default;
    const ros::Time& stamp() const { return m_stamp; }

protected:
    explicit Measurement(const ros::Time& stamp) : m_stamp(stamp) {}

private:
    ros::Time m_stamp;
};

class ImuMeasurement: public Measurement
{
public:
    explicit ImuMeasurement(const sensor_msgs::Imu& imu_msg);

    // Acceleration as measured by accelerometer.
    const ugl::Vector3& acceleration() const { return m_acc; }

    // Angular rate as measured by gyroscope.
    const ugl::Vector3& angular_rate() const { return m_rate; }

private:
    ugl::Vector3 m_acc;
    ugl::Vector3 m_rate;
};

class MocapMeasurement: public Measurement
{
public:
    explicit MocapMeasurement(const geometry_msgs::PoseStamped& pose_msg);

    // Pose measured by Mocap-system.
    const ugl::lie::Pose& pose() const { return m_pose; }

private:
    ugl::lie::Pose m_pose;
};

} // namespace invariant

#endif // INVARIANT_MEASUREMENT_H
