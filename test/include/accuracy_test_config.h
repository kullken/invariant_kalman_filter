#pragma once

#include <gtest/gtest.h>

#include "test_trajectories.h"
#include "test_filters.h"

namespace invariant::test
{

const auto test_trajectories = testing::Values(
    TestTrajectory{"StandStill 1s", getStandStillTrajectory()},
    TestTrajectory{"StandStill 10s", getStandStillTrajectory(10)},
    TestTrajectory{"Rotate 360 left, 1s", rotate_in_place(360, 1)},
    TestTrajectory{"Rotate 360 right, 1s", rotate_in_place(360, 1)},
    TestTrajectory{"Rotate 1080 right, 1s", rotate_in_place(1080, 1)},
    TestTrajectory{"Rotate 1080 right, 10s", rotate_in_place(1080, 10)},
    TestTrajectory{"Translate x: 1m; 1s", straight_line({1,0,0}, 1)},
    TestTrajectory{"Translate y: 1m; 1s", straight_line({0,1,0}, 1)},
    TestTrajectory{"Translate z: 1m; 1s", straight_line({0,0,1}, 1)},
    TestTrajectory{"Translate x: 10m; 1s", straight_line({10,0,0}, 1)},
    TestTrajectory{"Translate x: 10m; 10s", straight_line({10,0,0}, 10)}
);

const auto test_filters = testing::Values(
    TestFilter{"Default IEKF" , invariant::IEKF{}}
);

const auto test_configs = testing::Combine(
    test_trajectories,
    test_filters
);

} // namespace invariant::test
