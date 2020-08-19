#ifndef INVARIANT_MOCAP_SENSOR_MODEL_H
#define INVARIANT_MOCAP_SENSOR_MODEL_H

#include <ugl/math/vector.h>
#include <ugl/math/matrix.h>
#include <ugl/math/quaternion.h>
#include <ugl/lie_group/rotation.h>
#include <ugl/trajectory/trajectory.h>
#include <ugl/random/normal_distribution.h>

namespace invariant::test
{

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
    MocapSensorModel()
        : MocapSensorModel(MocapNoiseLevel::None) {}

    explicit MocapSensorModel(MocapNoiseLevel level, double frequency=100.0)
        : MocapSensorModel(ugl::trajectory::Trajectory{}, level, frequency) {}

    MocapSensorModel(const ugl::trajectory::Trajectory& trajectory, MocapNoiseLevel level, double frequency=100.0);

    double period() const
    {
        return period_;
    }

    MocapNoiseLevel noise_level() const
    {
        return noise_level_;
    }

    void set_trajectory(const ugl::trajectory::Trajectory& trajectory)
    {
        trajectory_ = trajectory;
    }

    /// Returns a position reading expressed in inertial frame.
    ugl::Vector3 get_pos_reading(double t) const;

    /// Returns a rotation reading expressed in inertial frame.
    ugl::lie::Rotation get_rot_reading(double t) const;

    /// Returns a rotation reading expressed in inertial frame in quaternion format.
    inline
    ugl::UnitQuaternion get_quat_reading(double t) const
    {
        return get_rot_reading(t).to_quaternion();
    }

private:
    ugl::trajectory::Trajectory trajectory_;
    MocapNoiseLevel noise_level_;
    double period_;

    ugl::random::NormalDistribution<3> position_noise_;
    ugl::random::NormalDistribution<3> rotation_noise_;
};

inline std::ostream& operator<<(std::ostream& os, const MocapNoiseLevel& level)
{
    switch (level)
    {
    case MocapNoiseLevel::None:
        return os << "None";
    case MocapNoiseLevel::Low:
        return os << "Low";
    case MocapNoiseLevel::High:
        return os << "High";
    }
}

inline std::ostream& operator<<(std::ostream& os, const MocapSensorModel& model)
{
    return os << "Mocap Noise: " << model.noise_level();
}

} // namespace invariant::test

#endif // INVARIANT_MOCAP_SENSOR_MODEL_H
