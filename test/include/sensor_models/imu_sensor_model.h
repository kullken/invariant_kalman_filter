#pragma once

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
    ImuSensorModel(const ugl::trajectory::Trajectory& trajectory, double frequency, ImuNoiseLevel level);

    double period() const
    {
        return period_;
    }

    /// Returns an accelerometer reading expressed in body frame.
    ugl::Vector3 get_accel_reading(double t) const;

    /// Returns an gyroscope reading expressed in body frame.
    ugl::Vector3 get_gyro_reading(double t) const;

private:
    const ugl::trajectory::Trajectory& trajectory_;
    const double period_;

    const ugl::random::NormalDistribution<3> accel_noise_;
    const ugl::random::NormalDistribution<3> gyro_noise_;

    static const ugl::Vector3 s_gravity;
};

} // namespace invariant::test
