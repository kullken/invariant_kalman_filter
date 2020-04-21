#pragma once

#include "iekf_types.h"
#include "linear_trajectory.h"
#include "angular_trajectory.h"

namespace invariant::test
{

class Trajectory
{
private:
    const LinearTrajectory m_linear_trajectory;
    const AngularTrajectory m_angular_trajectory;

public:
    Vector3 get_position(double t) const;
    Vector3 get_velocity(double t) const;
    Vector3 get_acceleration(double t) const;

    Rotation get_rotation(double t) const;
    Vector3 get_angular_velocity(double t) const;
};

}