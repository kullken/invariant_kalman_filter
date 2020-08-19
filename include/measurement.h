#ifndef INVARIANT_MEASUREMENT_H
#define INVARIANT_MEASUREMENT_H

#include <memory>

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
    virtual double get_time() const = 0;
};

class ImuMeasurement: public Measurement
{
public:
    ImuMeasurement(const sensor_msgs::Imu& data) : m_data(data) {}
    MeasurementType get_type() const override { return MeasurementType::imu; }
    double get_time() const override { return m_data.header.stamp.toSec(); }
    const sensor_msgs::Imu& get_data() const { return m_data; }
private:
    sensor_msgs::Imu m_data;
};

class MocapMeasurement: public Measurement
{
public:
    MocapMeasurement(const geometry_msgs::PoseStamped& data) : m_data(data) {}
    MeasurementType get_type() const override { return MeasurementType::imu; }
    double get_time() const override { return m_data.header.stamp.toSec(); }
    const geometry_msgs::PoseStamped& get_data() const { return m_data; }
private:
    geometry_msgs::PoseStamped m_data;
};

struct MeasurementCompare {
    bool operator()(const std::shared_ptr<Measurement> lhs, const std::shared_ptr<Measurement> rhs) const {
        return lhs->get_time() > rhs->get_time();
    }
};

} // namespace invariant

#endif // INVARIANT_MEASUREMENT_H
