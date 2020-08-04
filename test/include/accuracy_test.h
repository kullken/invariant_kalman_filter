#pragma once

#include <ostream>
#include <tuple>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <ugl/trajectory/trajectory.h>

#include "iekf.h"
#include "mekf.h"

#include "accuracy_test_config.h"
#include "test_trajectories.h"
#include "imu_sensor_model.h"
#include "mocap_sensor_model.h"

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
};

template<typename FilterType>
class AccuracyTest : public testing::TestWithParam<std::tuple<TestTrajectory, ImuSensorModel, MocapSensorModel>>
{
protected:
    AccuracyTest()
        : filter_()
        , trajectory_(std::get<0>(GetParam()).traj)
        , imu_(std::get<1>(GetParam()))
        , mocap_(std::get<2>(GetParam()))
    {
        imu_.set_trajectory(trajectory_);
        mocap_.set_trajectory(trajectory_);
    }

    Result compute_accuracy_impl();

private:
    FilterType filter_;
    ugl::trajectory::Trajectory trajectory_;
    ImuSensorModel imu_;
    MocapSensorModel mocap_;
};

class IekfTestSuite : public AccuracyTest<invariant::IEKF>
{
protected:
    IekfTestSuite() : AccuracyTest()
    {
    }

    Result compute_accuracy();
};

class MekfTestSuite : public AccuracyTest<mekf::MEKF>
{
protected:
    MekfTestSuite() : AccuracyTest()
    {
    }

    Result compute_accuracy();
};

std::ostream& operator<<(std::ostream& os, const Result& result);

} // namespace invariant::test
