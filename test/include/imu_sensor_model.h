#pragma once

#include <ugl/math/vector.h>
#include <ugl/trajectory/trajectory.h>

namespace invariant::test
{

/// A class for representing virtual IMU sensors.
/// TODO: Add noise and bias.
/// TODO: Enable arbitrary IMU coordinate system.
class ImuSensorModel
{
public:
    ImuSensorModel(const ugl::trajectory::Trajectory& trajectory) : trajectory_(trajectory) {}

    /// Returns an accelerometer reading expressed in body frame.
    ugl::Vector3 getAccReading(double t) const;

    /// Returns an gyroscope reading expressed in body frame.
    ugl::Vector3 getGyroReading(double t) const;

private:
    const ugl::trajectory::Trajectory& trajectory_;

    static const ugl::Vector3 s_gravity;
};

}
