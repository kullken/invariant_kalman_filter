#ifndef INVARIANT_ACCURACY_TEST_CONFIG_H
#define INVARIANT_ACCURACY_TEST_CONFIG_H

#include <gtest/gtest.h>

#include "test_trajectories.h"
#include "imu_sensor_model.h"
#include "mocap_sensor_model.h"

namespace invariant::test
{

inline
const auto test_trajectories_full = testing::Values(
    TestTrajectory{"StandStill 10s", getStandStillTrajectory(10)},
    TestTrajectory{"Rotate 360 left, 1s", rotate_in_place(360, 1)},
    TestTrajectory{"Rotate 360 right, 1s", rotate_in_place(-360, 1)},
    TestTrajectory{"Rotate 3600 left, 1s", rotate_in_place(3600, 1)},
    TestTrajectory{"Rotate 3600 left, 10s", rotate_in_place(3600, 10)},
    TestTrajectory{"ConstantVel xy: 10m; 10s", constant_velocity({1,1,0}, 10)},
    TestTrajectory{"ConstantVel  z: 10m; 10s", constant_velocity({0,0,1}, 10)},
    TestTrajectory{"Translate Quadratic xy: 1m; 1s", quadratic_translation({1,1,0}, 1)},
    TestTrajectory{"Translate Quadratic z: 1m; 1s", quadratic_translation({0,0,1}, 1)},
    TestTrajectory{"Translate Quadratic x: 10m; 1s", quadratic_translation({10,0,0}, 1)},
    TestTrajectory{"Translate Quadratic x: 10m; 10s", quadratic_translation({10,0,0}, 10)},
    TestTrajectory{"StartStop: {1,1,0}, 10s", start_stop({1,1,0}, 10)},
    TestTrajectory{"StartStop: {0,0,1}, 10s", start_stop({0,0,1}, 10)}
);

inline
const auto test_trajectories_partial = testing::Values(
    TestTrajectory{"StandStill 10s", getStandStillTrajectory(10)},
    TestTrajectory{"Rotate 3600 left, 10s", rotate_in_place(3600, 10)},
    TestTrajectory{"ConstantVel xy: 10m; 10s", constant_velocity({1,1,0}, 10)},
    TestTrajectory{"StartStop: {1,1,0}, 10s", start_stop({1,1,0}, 10)}
);

inline
const auto test_imu_models = testing::Values(
    ImuSensorModel{ImuNoiseLevel::None, 100.0},
    ImuSensorModel{ImuNoiseLevel::Mueller18, 100.0}
);

inline
const auto test_mocap_models = testing::Values(
    MocapSensorModel{MocapNoiseLevel::None, 100.0},
    MocapSensorModel{MocapNoiseLevel::Low, 100.0}
);

inline
const auto test_initial_errors = testing::Values(
    ugl::lie::ExtendedPose::Identity()
);

inline
const auto test_configs_full = testing::Combine(
    test_trajectories_full,
    test_imu_models,
    test_mocap_models,
    test_initial_errors
);

inline
const auto test_configs_partial = testing::Combine(
    test_trajectories_partial,
    test_imu_models,
    test_mocap_models,
    test_initial_errors
);

} // namespace invariant::test

#endif // INVARIANT_ACCURACY_TEST_CONFIG_H
