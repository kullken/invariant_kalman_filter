#pragma once

#include "iekf_types.h"

namespace invariant::test
{

class Trajectory
{
private:

public:
    Vector3 get_position(double t) const;
    Vector3 get_velocity(double t) const;
    Vector3 get_acceleration(double t) const;

    Rotation get_rotation(double t) const;
    Vector3 get_angular_velocity(double t) const;
};

}