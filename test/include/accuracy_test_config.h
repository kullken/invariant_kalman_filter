#pragma once

#include <vector>

#include <gtest/gtest.h>

#include <ugl/trajectory/trajectory.h>

#include "accuracy_test.h"

namespace invariant::test
{

auto test_configs = testing::Values(
    AccuracyTest::Config{"TestConfig_1", getStandStillTrajectory(), {}}
    // AccuracyTest::Config{"TestConfig_2", {}, {}}
    );

ugl::trajectory::Trajectory getStandStillTrajectory(double duration=1);

} // namespace invariant::test
