#pragma once

#include <vector>

#include "iekf_types.h"
#include "bezier.h"

namespace invariant::test
{

class LinearTrajectory
{
    using LinearTrajectorySegment = trajectory::Bezier<Vector3, 3>;

private:
    double m_duration = 0;

    std::vector<LinearTrajectorySegment> m_segments = {};

public:
    LinearTrajectory() = default;
    LinearTrajectory(std::vector<LinearTrajectorySegment> segments);

    Vector3 get_position(double t) const;
    Vector3 get_velocity(double t) const;
    Vector3 get_acceleration(double t) const;
};

}