#ifndef INVARIANT_MOCAP_SENSOR_MODEL_H
#define INVARIANT_MOCAP_SENSOR_MODEL_H

#include <iosfwd>

#include <ugl/math/vector.h>
#include <ugl/math/matrix.h>
#include <ugl/math/quaternion.h>
#include <ugl/lie_group/rotation.h>
#include <ugl/lie_group/pose.h>
#include <ugl/trajectory/trajectory.h>
#include <ugl/random/normal_distribution.h>

namespace invariant::test
{

struct MocapData
{
    ugl::lie::Pose pose;
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
    MocapSensorModel() = default;

    MocapSensorModel(MocapNoiseLevel level, double frequency);

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

    /// @return A position reading expressed in the inertial frame.
    ugl::Vector3 get_pos_reading(double t, const ugl::trajectory::Trajectory& trajectory) const;

    /// @return A rotation reading expressed in the inertial frame.
    ugl::lie::Rotation get_rot_reading(double t, const ugl::trajectory::Trajectory& trajectory) const;

private:
    MocapNoiseLevel noise_level_ = MocapNoiseLevel::None;
    double period_ = 0.01;

    ugl::random::NormalDistribution<3> position_noise_;
    ugl::random::NormalDistribution<3> rotation_noise_;
};

std::ostream& operator<<(std::ostream& os, const MocapNoiseLevel& level);

std::ostream& operator<<(std::ostream& os, const MocapSensorModel& model);

} // namespace invariant::test

#endif // INVARIANT_MOCAP_SENSOR_MODEL_H
