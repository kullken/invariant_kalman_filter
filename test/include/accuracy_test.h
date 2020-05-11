#pragma once

#include <string>

#include <gtest/gtest.h>

#include <ugl/trajectory/trajectory.h>

#include "iekf.h"

namespace invariant::test
{

struct AccuracyTestConfig
{
    std::string name = "";
    ugl::trajectory::Trajectory trajectory;
    // TODO: Add filter values and parameters.
    // TODO: Add sensor model and noise values.
};

struct AccuracyResult
{
    double final_accuracy = 0;
    double rmse_over_time = 0;
    // etc.
};

AccuracyResult compute_accuracy(const IEKF &filter, const ugl::trajectory::Trajectory &traj);

class AccuracyTestParam : public testing::TestWithParam<AccuracyTestConfig>
{
protected:
    ugl::trajectory::Trajectory trajectory_;
    invariant::IEKF iekf_{};

protected:
    AccuracyTestParam() : trajectory_(GetParam().trajectory)
    {
    }
};

} // namespace invariant::test
