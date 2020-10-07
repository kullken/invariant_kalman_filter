#ifndef INVARIANT_ACCURACY_TEST_H
#define INVARIANT_ACCURACY_TEST_H

#include <ostream>
#include <tuple>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <ugl/lie_group/extended_pose.h>
#include <ugl/trajectory/trajectory.h>
#include <ugl/random/seed.h>

#include "iekf.h"
#include "mekf.h"

#include "accuracy_test_config.h"
#include "test_trajectories.h"
#include "virtual_sensor.h"

namespace invariant::test
{

struct Result
{
    double position_rmse = 0;
    double velocity_rmse = 0;
    double rotation_rmse = 0;

    std::vector<double> times;
    std::vector<double> position_errors;
    std::vector<double> velocity_errors;
    std::vector<double> rotation_errors;

    std::vector<ugl::lie::ExtendedPose> estimates;
    std::vector<ugl::lie::ExtendedPose> ground_truth;
};

template<typename FilterType>
class AccuracyTest : public testing::TestWithParam<std::tuple<TestTrajectory, std::vector<VirtualSensor>, ugl::lie::ExtendedPose>>
{
protected:
    AccuracyTest()
        : filter_()
        , trajectory_(std::get<0>(GetParam()).traj)
        , sensors_(std::get<1>(GetParam()))
    {
        ugl::random::set_seed(117);

        const auto initial_error = std::get<2>(GetParam());
        const auto initial_state = trajectory_.get_extended_pose(0.0) * initial_error;
        filter_.set_state(initial_state);
    }

protected:
    FilterType filter_;
    ugl::trajectory::Trajectory trajectory_;
    std::vector<VirtualSensor> sensors_;
};

class IekfTestSuite : public AccuracyTest<invariant::IEKF>
{
protected:
    IekfTestSuite() : AccuracyTest()
    {
    }

    Result compute_accuracy();
};

class MekfTestSuite : public AccuracyTest<invariant::MEKF>
{
protected:
    MekfTestSuite() : AccuracyTest()
    {
    }

    Result compute_accuracy();
};

std::ostream& operator<<(std::ostream& os, const Result& result);

} // namespace invariant::test

#endif // INVARIANT_ACCURACY_TEST_H
