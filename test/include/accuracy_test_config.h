#pragma once

#include <gtest/gtest.h>

#include "test_trajectories.h"
#include "imu_sensor_model.h"
#include "mocap_sensor_model.h"

namespace invariant::test
{

const auto test_trajectories = testing::Values(
    TestTrajectory{"StandStill 10s", getStandStillTrajectory(10)},
    TestTrajectory{"Rotate 360 left, 1s", rotate_in_place(360, 1)},
    TestTrajectory{"Rotate 360 right, 1s", rotate_in_place(360, 1)},
    TestTrajectory{"Rotate 3600 right, 1s", rotate_in_place(3600, 1)},
    TestTrajectory{"Rotate 3600 right, 10s", rotate_in_place(3600, 10)},
    TestTrajectory{"Translate Linear xy: 10m; 10s", straight_line({10,10,0}, 10)},
    TestTrajectory{"Translate Linear z: 10m; 10s", straight_line({0,0,10}, 10)},
    TestTrajectory{"Translate Quadratic xy: 1m; 1s", quadratic_translation({1,1,0}, 1)},
    TestTrajectory{"Translate Quadratic z: 1m; 1s", quadratic_translation({0,0,1}, 1)},
    TestTrajectory{"Translate Quadratic x: 10m; 1s", quadratic_translation({10,0,0}, 1)},
    TestTrajectory{"Translate Quadratic x: 10m; 10s", quadratic_translation({10,0,0}, 10)}
);

const auto test_imu_models = testing::Values(
    ImuSensorModel{ImuNoiseLevel::None},
    ImuSensorModel{ImuNoiseLevel::Mueller18}
);

const auto test_mocap_models = testing::Values(
    MocapSensorModel{MocapNoiseLevel::None}
);

const auto test_configs = testing::Combine(
    test_trajectories,
    test_imu_models,
    test_mocap_models
);

} // namespace invariant::test
