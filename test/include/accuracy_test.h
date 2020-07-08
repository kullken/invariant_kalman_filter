#pragma once

#include <ostream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <ugl/trajectory/trajectory.h>

#include "iekf.h"

#include "accuracy_test_config.h"
#include "test_trajectories.h"
#include "test_filters.h"

namespace invariant::test
{

class AccuracyTest : public testing::Test
{
protected:
    ugl::trajectory::Trajectory trajectory_;

public:

    struct Result
    {
        double position_rmse = 0;
        double velocity_rmse = 0;
        double rotation_rmse = 0;

        std::vector<double> times;
        std::vector<double> position_errors;
        std::vector<double> velocity_errors;
        std::vector<double> rotation_errors;
    };

protected:
    AccuracyTest::Result compute_accuracy(IEKF filter, const ugl::trajectory::Trajectory &traj);
};

class IekfTestSuite 
    : public AccuracyTest
    , public testing::WithParamInterface<std::tuple<TestTrajectory, TestFilter>>
{
protected:
    invariant::IEKF filter_;

protected:

    IekfTestSuite() : filter_(std::get<1>(GetParam()).filter)
    {
        trajectory_ = std::get<0>(GetParam()).traj;
    }
};

std::ostream& operator<<(std::ostream& os, const AccuracyTest::Result& result);

} // namespace invariant::test
