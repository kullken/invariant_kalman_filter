#include "mocap_sensor_model.h"

#include <exception>
#include <ostream>

#include <ugl/math/vector.h>
#include <ugl/math/matrix.h>
#include <ugl/lie_group/pose.h>

namespace invariant::test
{

static ugl::Matrix<6,6> get_covariance(MocapNoiseLevel level);

MocapSensorModel::MocapSensorModel(MocapNoiseLevel level, double frequency, const ugl::lie::Pose& offset)
    : noise_level_(level)
    , period_(1.0/frequency)
    , noise_(get_covariance(level))
    , model_(offset, get_covariance(level)) /// TODO: What value to assign when noise level == None?
{
}

MocapData MocapSensorModel::get_data(double t, const ugl::trajectory::Trajectory& trajectory) const
{
    return MocapData{get_pose_reading(t, trajectory), model_};
}

ugl::lie::Pose MocapSensorModel::get_pose_reading(double t, const ugl::trajectory::Trajectory& trajectory) const
{

    return model_.h(trajectory.get_extended_pose(t), noise_.sample());
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

static ugl::Matrix<6,6> get_covariance(MocapNoiseLevel level)
{
    ugl::Matrix<6,6> covar = ugl::Matrix<6,6>::Identity();
    switch (level)
    {
    case MocapNoiseLevel::None:
        covar.block<3,3>(0,0) = rotation_covar<MocapNoiseLevel::None>();
        covar.block<3,3>(3,3) = position_covar<MocapNoiseLevel::None>();
        return covar;
    case MocapNoiseLevel::Low:
        covar.block<3,3>(0,0) = rotation_covar<MocapNoiseLevel::Low>();
        covar.block<3,3>(3,3) = position_covar<MocapNoiseLevel::Low>();
        return covar;
    default:
        throw std::logic_error("The asked for MocapNoiseLevel is not yet implemented.");
    }
}

} // namespace invariant::test
