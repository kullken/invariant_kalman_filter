#include "mocap_sensor_model.h"

#include <exception>
#include <ostream>

#include <ugl/math/vector.h>
#include <ugl/math/matrix.h>
#include <ugl/lie_group/rotation.h>
#include <ugl/lie_group/pose.h>

namespace invariant::test
{

static ugl::Matrix3 get_position_covar(MocapNoiseLevel level);
static ugl::Matrix3 get_rotation_covar(MocapNoiseLevel level);

MocapSensorModel::MocapSensorModel(MocapNoiseLevel level, double frequency)
    : noise_level_(level)
    , period_(1.0/frequency)
    , position_noise_(get_position_covar(level))
    , rotation_noise_(get_rotation_covar(level))
{
}

ugl::lie::Pose MocapSensorModel::get_pose_reading(double t, const ugl::trajectory::Trajectory& trajectory) const
{
    return ugl::lie::Pose{get_rot_reading(t, trajectory), get_pos_reading(t, trajectory)};
}

ugl::Vector3 MocapSensorModel::get_pos_reading(double t, const ugl::trajectory::Trajectory& trajectory) const
{
    return trajectory.get_position(t) + position_noise_.sample();
}

ugl::lie::Rotation MocapSensorModel::get_rot_reading(double t, const ugl::trajectory::Trajectory& trajectory) const
{
    return trajectory.get_rotation(t) * ugl::lie::SO3::exp(rotation_noise_.sample());
}

std::ostream& operator<<(std::ostream& os, const MocapNoiseLevel& level)
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

std::ostream& operator<<(std::ostream& os, const MocapSensorModel& model)
{
    return os << "Mocap Noise: " << model.noise_level();
}

template<MocapNoiseLevel level>
static ugl::Matrix3 position_covar();

template<>
ugl::Matrix3 position_covar<MocapNoiseLevel::None>()
{
    return ugl::Matrix3::Zero();
}

template<>
ugl::Matrix3 position_covar<MocapNoiseLevel::Low>()
{
    constexpr double stddev = 0.01;  // [m]
    constexpr double variance = stddev*stddev;
    return ugl::Matrix3::Identity() * variance;
}

static ugl::Matrix3 get_position_covar(MocapNoiseLevel level)
{
    switch (level)
    {
    case MocapNoiseLevel::None:
        return position_covar<MocapNoiseLevel::None>();
    case MocapNoiseLevel::Low:
        return position_covar<MocapNoiseLevel::Low>();
    default:
        throw std::logic_error("The asked for MocapNoiseLevel is not yet implemented.");
    }
}

template<MocapNoiseLevel level>
static ugl::Matrix3 rotation_covar();

template<>
ugl::Matrix3 rotation_covar<MocapNoiseLevel::None>()
{
    return ugl::Matrix3::Zero();
}

template<>
ugl::Matrix3 rotation_covar<MocapNoiseLevel::Low>()
{
    constexpr double stddev = 0.01;  // [rad]
    constexpr double variance = stddev*stddev;
    return ugl::Matrix3::Identity() * variance;
}

static ugl::Matrix3 get_rotation_covar(MocapNoiseLevel level)
{
    switch (level)
    {
    case MocapNoiseLevel::None:
        return rotation_covar<MocapNoiseLevel::None>();
    case MocapNoiseLevel::Low:
        return rotation_covar<MocapNoiseLevel::Low>();
    default:
        throw std::logic_error("The asked for MocapNoiseLevel is not yet implemented.");
    }
}

} // namespace invariant::test
