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
        // TODO: Add filter values and parameters.
        // TODO: Add sensor model and noise values.
    };

    struct Result
    {
        double final_accuracy = 0;
        double rmse_over_time = 0;
        // etc.
    };
};

class AccuracyTestParam : public AccuracyTest,
                     public testing::WithParamInterface<AccuracyTest::Config>
{
protected:
    ugl::trajectory::Trajectory trajectory_;
    invariant::IEKF iekf_{};

protected:
    AccuracyTestParam() : trajectory_(GetParam().trajectory)
    {
    }
};

AccuracyTest::Result compute_accuracy(const IEKF &filter, const ugl::trajectory::Trajectory &traj);

} // namespace invariant::test
