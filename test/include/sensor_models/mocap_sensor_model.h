#ifndef INVARIANT_MOCAP_SENSOR_MODEL_H
#define INVARIANT_MOCAP_SENSOR_MODEL_H

#include <iosfwd>

#include <ugl/lie_group/pose.h>
#include <ugl/trajectory/trajectory.h>
#include <ugl/random/normal_distribution.h>

#include "mocap_model.h"

namespace invariant::test
{

struct MocapData
{
    ugl::lie::Pose measurement;
    MocapModel model;
};

enum class MocapNoiseLevel
{
    None,
    Low,
    High
};

/// A class for representing virtual Motion-capture sensors.
/// TODO: Add bias.
class MocapSensorModel
{
public:
    MocapSensorModel(MocapNoiseLevel level, double frequency, const ugl::lie::Pose& offset=ugl::lie::Pose::Identity());

    double period() const
    {
        return period_;
    }

    MocapNoiseLevel noise_level() const
    {
        return noise_level_;
    }

    /// @brief Generate sensor data from a trajectory
    /// @param t the time at which to generate data
    /// @param trajectory the trajectory from which to generate data
    /// @return The sensor data
    MocapData get_data(double t, const ugl::trajectory::Trajectory& trajectory) const;

    /// @return A pose reading expressed in the inertial frame.
    ugl::lie::Pose get_pose_reading(double t, const ugl::trajectory::Trajectory& trajectory) const;

private:
    MocapNoiseLevel noise_level_ = MocapNoiseLevel::None;
    double period_ = 0.01;

    ugl::random::NormalDistribution<6> noise_;
    MocapModel model_;
};

std::ostream& operator<<(std::ostream& os, const MocapNoiseLevel& level);

std::ostream& operator<<(std::ostream& os, const MocapSensorModel& model);

} // namespace invariant::test

#endif // INVARIANT_MOCAP_SENSOR_MODEL_H
