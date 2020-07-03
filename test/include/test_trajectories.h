#pragma once

#include <iostream>
#include <string>

#include <ugl/math/vector.h>
#include <ugl/trajectory/trajectory.h>


namespace invariant::test
{

struct TestTrajectory
{
    std::string description;
    ugl::trajectory::Trajectory traj;
};

inline std::ostream& operator<<(std::ostream& os, const TestTrajectory& param)
{
    return os << param.description;
}

ugl::trajectory::Trajectory getStandStillTrajectory(double duration=1.0);

ugl::trajectory::Trajectory rotate_in_place(double degrees, double duration=1.0);

ugl::trajectory::Trajectory straight_line(ugl::Vector3 delta, double duration=1.0);

} // namespace invariant::test