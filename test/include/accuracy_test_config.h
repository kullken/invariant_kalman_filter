#pragma once

#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <ugl/trajectory/trajectory.h>

#include "iekf.h"

namespace invariant::test
{

ugl::trajectory::Trajectory getStandStillTrajectory(double duration=1);

struct TrajectoryNamed
{
    std::string name;
    ugl::trajectory::Trajectory traj;
};

inline std::ostream& operator<<(std::ostream& os, const TrajectoryNamed& param)
{
    return os << param.name;
}

const auto test_trajectories = testing::Values(
    TrajectoryNamed{"StandStill 1s", getStandStillTrajectory()},
    TrajectoryNamed{"StandStill 2s", getStandStillTrajectory(2)}
);

struct FilterNamed
{
    std::string name;
    invariant::IEKF filter;
};

inline std::ostream& operator<<(std::ostream& os, const FilterNamed& param)
{
    return os << param.name;
}

const auto test_filters = testing::Values(
    FilterNamed{"Default IEKF" , invariant::IEKF{}}
);

const auto test_configs = testing::Combine(
    test_trajectories,
    test_filters
);

} // namespace invariant::test
