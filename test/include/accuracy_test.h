#pragma once

#include <ostream>
#include <tuple>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <ugl/trajectory/trajectory.h>

#include "iekf.h"

#include "accuracy_test_config.h"
#include "test_trajectories.h"
#include "imu_sensor_model.h"

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

        std::vector<double> times;
        std::vector<double> position_errors;
        std::vector<double> velocity_errors;
        std::vector<double> rotation_errors;
    };

// protected:
//     AccuracyTest::Result compute_accuracy(IEKF filter, const ugl::trajectory::Trajectory &traj);
};

class IekfTestSuite 
    : public AccuracyTest
    , public testing::WithParamInterface<std::tuple<TestTrajectory, ImuSensorModel>>
{
protected:

    IekfTestSuite()
        : trajectory_(std::get<0>(GetParam()).traj)
        , imu_(std::get<1>(GetParam()))
    {
        imu_.set_trajectory(trajectory_);
    }

    AccuracyTest::Result compute_accuracy(IEKF filter, const ugl::trajectory::Trajectory &traj);

protected:
    invariant::IEKF filter_;
    ugl::trajectory::Trajectory trajectory_;
    ImuSensorModel imu_;
};

std::ostream& operator<<(std::ostream& os, const AccuracyTest::Result& result);

} // namespace invariant::test
