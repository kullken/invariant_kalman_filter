#pragma once

#include <string>

#include <gtest/gtest.h>

#include <ugl/trajectory/trajectory.h>

#include "iekf.h"

namespace invariant::test
{

class AccuracyTest : public testing::Test
{
public:
    struct Config
    {
        std::string name = "";
        ugl::trajectory::Trajectory trajectory;
        invariant::IEKF filter;
        // TODO: Configure filter noise parameters.
        // TODO: Add sensor model and noise values.
    };

    struct Result
    {
        double final_accuracy = 0;
        double rmse_over_time = 0;
        // etc.
    };

protected:
    AccuracyTest::Result compute_accuracy(IEKF filter, const ugl::trajectory::Trajectory &traj);
};

class AccuracyTestParam : public AccuracyTest,
                     public testing::WithParamInterface<AccuracyTest::Config>
{
protected:
    const ugl::trajectory::Trajectory trajectory_;
    invariant::IEKF filter_;

protected:
    AccuracyTestParam()
        : trajectory_(GetParam().trajectory)
        , filter_(GetParam().filter)
    {
    }
};

} // namespace invariant::test
