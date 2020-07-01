#pragma once

#include <gtest/gtest.h>

#include "test_trajectories.h"
#include "test_filters.h"

namespace invariant::test
{

const auto test_trajectories = testing::Values(
    TrajectoryNamed{"StandStill 1s", getStandStillTrajectory()},
    TrajectoryNamed{"StandStill 2s", getStandStillTrajectory(2)},
    TrajectoryNamed{"StandStill 3s", getStandStillTrajectory(3)}
);

const auto test_filters = testing::Values(
    FilterNamed{"Default IEKF" , invariant::IEKF{}}
);

const auto test_configs = testing::Combine(
    test_trajectories,
    test_filters
);

} // namespace invariant::test
