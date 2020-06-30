#pragma once

#include <ostream>
#include <string>
#include <vector>

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

inline std::ostream& operator<<(std::ostream& os, const AccuracyTest::Config& config)
{
    return os << config.name;
}

std::ostream& operator<<(std::ostream& os, const AccuracyTest::Result& result);

} // namespace invariant::test
