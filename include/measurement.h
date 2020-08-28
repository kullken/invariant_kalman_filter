#ifndef INVARIANT_MEASUREMENT_H
#define INVARIANT_MEASUREMENT_H

#include <ros/time.h>

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>

namespace invariant
{

class Measurement
{
public:
    virtual ~Measurement() = default;
    const ros::Time& stamp() const { return m_stamp; }

protected:
    Measurement(const ros::Time& stamp) : m_stamp(stamp) {}

private:
    ros::Time m_stamp;
};

class ImuMeasurement: public Measurement
{
public:
    ImuMeasurement(const sensor_msgs::Imu& data)
        : Measurement(data.header.stamp)
        , m_data(data)
    {
    }

    const sensor_msgs::Imu& get_data() const { return m_data; }

private:
    sensor_msgs::Imu m_data;
};

class MocapMeasurement: public Measurement
{
public:
    MocapMeasurement(const geometry_msgs::PoseStamped& data)
        : Measurement(data.header.stamp)
        , m_data(data)
    {
    }

    const geometry_msgs::PoseStamped& get_data() const { return m_data; }

private:
    geometry_msgs::PoseStamped m_data;
};

} // namespace invariant

#endif // INVARIANT_MEASUREMENT_H
