#ifndef INVARIANT_IMU_SENSOR_MODEL_H
#define INVARIANT_IMU_SENSOR_MODEL_H

#include <iosfwd>

#include <ugl/math/vector.h>
#include <ugl/trajectory/trajectory.h>
#include <ugl/random/normal_distribution.h>

namespace invariant::test
{

enum class ImuNoiseLevel
{
    None,
    Low,
    High,
    Mueller18,
};

/// A class for representing virtual IMU sensors.
/// TODO: Add bias?.
/// TODO: Enable arbitrary IMU coordinate system.
class ImuSensorModel
{
public:
    ImuSensorModel() = default;

    ImuSensorModel(ImuNoiseLevel level, double frequency);

    double period() const
    {
        return period_;
    }

    ImuNoiseLevel noise_level() const
    {
        return noise_level_;
    }

    /// @return An accelerometer reading expressed in the body frame.
    ugl::Vector3 get_accel_reading(double t, const ugl::trajectory::Trajectory& trajectory) const;

    /// @return An gyroscope reading expressed in the body frame.
    ugl::Vector3 get_gyro_reading(double t, const ugl::trajectory::Trajectory& trajectory) const;

private:
    ImuNoiseLevel noise_level_ = ImuNoiseLevel::None;
    double period_ = 0.01;

    ugl::random::NormalDistribution<3> accel_noise_;
    ugl::random::NormalDistribution<3> gyro_noise_;

    static const ugl::Vector3 s_gravity;
};

std::ostream& operator<<(std::ostream& os, const ImuNoiseLevel& level);

std::ostream& operator<<(std::ostream& os, const ImuSensorModel& model);

} // namespace invariant::test

#endif // INVARIANT_IMU_SENSOR_MODEL_H
