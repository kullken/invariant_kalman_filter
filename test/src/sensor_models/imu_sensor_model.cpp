#include "imu_sensor_model.h"

#include <exception>
#include <ostream>

#include <ugl/math/vector.h>
#include <ugl/math/matrix.h>
#include <ugl/lie_group/rotation.h>

namespace invariant::test
{

static ugl::Matrix3 get_accel_covar(ImuNoiseLevel level);
static ugl::Matrix3 get_gyro_covar(ImuNoiseLevel level);

const ugl::Vector3 ImuSensorModel::s_gravity{0.0, 0.0, -9.82};

ImuSensorModel::ImuSensorModel(ImuNoiseLevel level, double frequency)
    : noise_level_(level)
    , period_(1.0/frequency)
    , accel_noise_(get_accel_covar(level))
    , gyro_noise_(get_gyro_covar(level))
{
}

ImuData ImuSensorModel::get_data(double t, const ugl::trajectory::Trajectory& trajectory) const
{
    return ImuData{period(), get_accel_reading(t, trajectory), get_gyro_reading(t, trajectory)};
}

ugl::Vector3 ImuSensorModel::get_accel_reading(double t, const ugl::trajectory::Trajectory& trajectory) const
{
    ugl::Vector3 acc = trajectory.get_acceleration(t);
    ugl::lie::Rotation R_inv = trajectory.get_rotation(t).inverse();
    return R_inv * (acc - s_gravity) + accel_noise_.sample();
}

ugl::Vector3 ImuSensorModel::get_gyro_reading(double t, const ugl::trajectory::Trajectory& trajectory) const
{
    ugl::Vector3 gyro = trajectory.get_angular_velocity(t);
    ugl::lie::Rotation R_inv = trajectory.get_rotation(t).inverse();
    return R_inv * gyro  + gyro_noise_.sample();
}

std::ostream& operator<<(std::ostream& os, const ImuNoiseLevel& level)
{
    switch (level)
    {
    case ImuNoiseLevel::None:
        return os << "None";
    case ImuNoiseLevel::Low:
        return os << "Low";
    case ImuNoiseLevel::High:
        return os << "High";
    case ImuNoiseLevel::Mueller18:
        return os << "Mueller18";
    }
}

std::ostream& operator<<(std::ostream& os, const ImuSensorModel& model)
{
    return os << "IMU Noise: " << model.noise_level();
}

template<ImuNoiseLevel level>
static ugl::Matrix3 accel_covar();

template<>
ugl::Matrix3 accel_covar<ImuNoiseLevel::None>()
{
    return ugl::Matrix3::Zero();
}

template<>
ugl::Matrix3 accel_covar<ImuNoiseLevel::Low>()
{
    constexpr double sigma_accel = 2.5;  // [m/s^2]
    constexpr double variance = sigma_accel*sigma_accel;
    return ugl::Matrix3::Identity() * variance;
}

template<>
ugl::Matrix3 accel_covar<ImuNoiseLevel::Mueller18>()
{
    // Value taken from Mueller et al. (2018).
    constexpr double sigma_accel = 5;  // [m/s^2]
    constexpr double variance = sigma_accel*sigma_accel;
    return ugl::Matrix3::Identity() * variance;
}

static ugl::Matrix3 get_accel_covar(ImuNoiseLevel level)
{
    switch (level)
    {
    case ImuNoiseLevel::None:
        return accel_covar<ImuNoiseLevel::None>();
    case ImuNoiseLevel::Low:
        return accel_covar<ImuNoiseLevel::Low>();
    case ImuNoiseLevel::Mueller18:
        return accel_covar<ImuNoiseLevel::Mueller18>();
    default:
        throw std::logic_error("The asked for ImuNoiseLevel is not yet implemented for accelerometer.");
    }
}

template<ImuNoiseLevel level>
static ugl::Matrix3 gyro_covar();

template<>
ugl::Matrix3 gyro_covar<ImuNoiseLevel::None>()
{
    return ugl::Matrix3::Zero();
}

template<>
ugl::Matrix3 gyro_covar<ImuNoiseLevel::Low>()
{
    constexpr double sigma_gyro = 0.05;  // [rad/s]
    constexpr double variance = sigma_gyro*sigma_gyro;
    return ugl::Matrix3::Identity() * variance;
}

template<>
ugl::Matrix3 gyro_covar<ImuNoiseLevel::Mueller18>()
{
    // Value taken from Mueller et al. (2018).
    constexpr double sigma_gyro = 0.1;  // [rad/s]
    constexpr double variance = sigma_gyro*sigma_gyro;
    return ugl::Matrix3::Identity() * variance;
}

static ugl::Matrix3 get_gyro_covar(ImuNoiseLevel level)
{
    switch (level)
    {
    case ImuNoiseLevel::None:
        return gyro_covar<ImuNoiseLevel::None>();
    case ImuNoiseLevel::Low:
        return gyro_covar<ImuNoiseLevel::Low>();
    case ImuNoiseLevel::Mueller18:
        return gyro_covar<ImuNoiseLevel::Mueller18>();
    default:
        throw std::logic_error("The asked for ImuNoiseLevel is not yet implemented for gyroscope.");
    }
}

} // namespace invariant::test
