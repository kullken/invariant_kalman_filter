#pragma once

#include <vector>

#include "iekf_types.h"

namespace invariant::test
{

class AngularTrajectorySegment
{

};

class AngularTrajectory
{
private:
    std::vector<AngularTrajectorySegment> m_segments;

public:
    Rotation get_rotation(double t) const;
    Vector3 get_angular_velocity(double t) const;
};

}