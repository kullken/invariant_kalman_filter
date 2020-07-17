#include "imu_sensor_model.h"

#include <exception>

#include <ugl/math/vector.h>
#include <ugl/math/matrix.h>

namespace invariant::test
{

static ugl::Matrix3 get_accel_covar(ImuNoiseLevel level);
static ugl::Matrix3 get_gyro_covar(ImuNoiseLevel level);

const ugl::Vector3 ImuSensorModel::s_gravity{0.0, 0.0, -9.82};

ImuSensorModel::ImuSensorModel(const ugl::trajectory::Trajectory& trajectory, double frequency, ImuNoiseLevel level)
    : trajectory_(trajectory) 
    , period_(1.0/frequency)
    , noise_level_(level)
    , accel_noise_(get_accel_covar(level))
    , gyro_noise_(get_gyro_covar(level))
{
}

ugl::Vector3 ImuSensorModel::get_accel_reading(double t) const
{
    ugl::Vector3 acc = trajectory_.get_acceleration(t);
    ugl::Rotation R_inv = trajectory_.get_rotation(t).inverse();
    return R_inv * (acc - s_gravity) + accel_noise_.sample();
}

ugl::Vector3 ImuSensorModel::get_gyro_reading(double t) const
{
    ugl::Vector3 gyro = trajectory_.get_angular_velocity(t);
    ugl::Rotation R_inv = trajectory_.get_rotation(t).inverse();
    return R_inv * gyro  + gyro_noise_.sample();
}

template<ImuNoiseLevel level>
static ugl::Matrix3 accel_covar();

template<>
ugl::Matrix3 accel_covar<ImuNoiseLevel::None>()
{
    return ugl::Matrix3::Zero();
}

template<>
ugl::Matrix3 accel_covar<ImuNoiseLevel::Mueller18>()
{
    // Value taken from Mueller et al. (2018).
    constexpr double sigma_accel = 5;  // m/s^2
    constexpr double variance = sigma_accel*sigma_accel;
    return ugl::Vector3{variance, variance, variance}.asDiagonal();
}

static ugl::Matrix3 get_accel_covar(ImuNoiseLevel level)
{
    switch (level)
    {
    case ImuNoiseLevel::None:
        return accel_covar<ImuNoiseLevel::None>();
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
ugl::Matrix3 gyro_covar<ImuNoiseLevel::Mueller18>()
{
    // Value taken from Mueller et al. (2018).
    constexpr double sigma_gyro = 0.1;  // rad/s
    constexpr double variance = sigma_gyro*sigma_gyro;
    return ugl::Vector3{variance, variance, variance}.asDiagonal();
}

static ugl::Matrix3 get_gyro_covar(ImuNoiseLevel level)
{
    switch (level)
    {
    case ImuNoiseLevel::None:
        return gyro_covar<ImuNoiseLevel::None>();
    case ImuNoiseLevel::Mueller18:
        return gyro_covar<ImuNoiseLevel::Mueller18>();
    default:
        throw std::logic_error("The asked for ImuNoiseLevel is not yet implemented for gyroscope.");
    }
}

} // namespace invariant::test
