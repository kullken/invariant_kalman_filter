#pragma once

#include <ugl/math/vector.h>
#include <ugl/math/matrix.h>
#include <ugl/trajectory/trajectory.h>

namespace invariant::test
{

/// A class for representing virtual Motion-capture sensors.
/// TODO: Add noise and bias.
class MocapSensorModel
{
public:
    MocapSensorModel(const ugl::trajectory::Trajectory& trajectory, double frequency)
        : trajectory_(trajectory) 
        , period_(1.0/frequency)
    {}

    double period() 
    {
        return period_;
    }

    /// Returns a position reading expressed in inertial frame.
    ugl::Vector3 get_pos_reading(double t) const;

    /// Returns a rotation reading expressed in inertial frame.
    ugl::Rotation get_rot_reading(double t) const;

private:
    const ugl::trajectory::Trajectory& trajectory_;
    const double period_;
};

}
