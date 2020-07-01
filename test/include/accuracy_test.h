#pragma once

#include <ostream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <ugl/trajectory/trajectory.h>

#include "iekf.h"

#include "accuracy_test_config.h"

namespace invariant::test
{

class AccuracyTest : public testing::Test
{
public:

    struct Result
    {
        double position_rmse = 0;
        double velocity_rmse = 0;
        double rotation_rmse = 0;

        std::vector<double> position_errors;
        std::vector<double> velocity_errors;
        std::vector<double> rotation_errors;
    };

protected:
    AccuracyTest::Result compute_accuracy(IEKF filter, const ugl::trajectory::Trajectory &traj);
};

class AccuracyTestParam 
    : public AccuracyTest
    , public testing::WithParamInterface<std::tuple<TrajectoryNamed, FilterNamed>>
{
protected:
    const ugl::trajectory::Trajectory trajectory_;
    invariant::IEKF filter_;

protected:

    AccuracyTestParam()
        : trajectory_(std::get<0>(GetParam()).traj)
        , filter_(std::get<1>(GetParam()).filter)
    {
    }
};

std::ostream& operator<<(std::ostream& os, const AccuracyTest::Result& result);

} // namespace invariant::test
