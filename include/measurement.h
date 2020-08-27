#ifndef INVARIANT_MEASUREMENT_H
#define INVARIANT_MEASUREMENT_H

#include <memory>

#include <ros/time.h>

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>

namespace invariant
{

enum class MeasurementType
{
    imu,
    mocap
};

class Measurement
{
public:
    virtual ~Measurement() = default;
    virtual MeasurementType get_type() const = 0;

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

    MeasurementType get_type() const override { return MeasurementType::imu; }
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

    MeasurementType get_type() const override { return MeasurementType::mocap; }
    const geometry_msgs::PoseStamped& get_data() const { return m_data; }
private:
    geometry_msgs::PoseStamped m_data;
};

struct MeasurementCompare {
    bool operator()(const std::shared_ptr<Measurement> lhs, const std::shared_ptr<Measurement> rhs) const {
        return lhs->stamp() > rhs->stamp();
    }
};

} // namespace invariant

#endif // INVARIANT_MEASUREMENT_H
