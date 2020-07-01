#pragma once

#include <iostream>
#include <string>

#include <ugl/trajectory/trajectory.h>


namespace invariant::test
{

struct TrajectoryNamed
{
    std::string name;
    ugl::trajectory::Trajectory traj;
};

inline std::ostream& operator<<(std::ostream& os, const TrajectoryNamed& param)
{
    return os << param.name;
}

ugl::trajectory::Trajectory getStandStillTrajectory(double duration=1.0);

} // namespace invariant::test