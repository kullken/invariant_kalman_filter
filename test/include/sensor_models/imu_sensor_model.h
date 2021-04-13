#ifndef INVARIANT_IMU_SENSOR_MODEL_H
#define INVARIANT_IMU_SENSOR_MODEL_H

#include <iosfwd>

#include <ugl/math/vector.h>
#include <ugl/trajectory/trajectory.h>
#include <ugl/random/normal_distribution.h>

#include "imu_model.h"

namespace invariant::test
{

struct ImuData
{
    double dt;
    ugl::Vector3 acc;
    ugl::Vector3 rate;
    ImuModel model;
};

enum class ImuNoiseLevel
{
    None,
    Low,
    High,
    Mueller18,
    Custom,
};

/// A class for representing virtual IMU sensors.
/// TODO: Add bias?.
/// TODO: Enable arbitrary IMU coordinate system.
class ImuSensorModel
{
public:
    ImuSensorModel(ImuNoiseLevel level, double frequency);

    ImuSensorModel(const ugl::Matrix<6,6>& covariance, double frequency);

    ImuSensorModel(const ugl::Matrix<6,6>& true_covariance, const ugl::Matrix<6,6>& believed_covariance, double frequency);

    double period() const
    {
        return m_period;
    }

    ImuNoiseLevel noise_level() const
    {
        return m_noise_level;
    }

    /// @brief Generate sensor data from a trajectory
    /// @param t the time at which to generate data
    /// @param trajectory the trajectory from which to generate data
    /// @return The sensor data
    ImuData get_data(double t, const ugl::trajectory::Trajectory& trajectory) const;

    /// @return An accelerometer reading expressed in the body frame.
    ugl::Vector3 get_accel_reading(double t, const ugl::trajectory::Trajectory& trajectory) const;

    /// @return An gyroscope reading expressed in the body frame.
    ugl::Vector3 get_gyro_reading(double t, const ugl::trajectory::Trajectory& trajectory) const;

private:
    ImuNoiseLevel m_noise_level = ImuNoiseLevel::None;
    double m_period = 0.01;

    ugl::random::NormalDistribution<3> m_accel_noise;
    ugl::random::NormalDistribution<3> m_gyro_noise;
    ImuModel m_imu_model;

    static const ugl::Vector3 s_gravity;
};

std::ostream& operator<<(std::ostream& os, const ImuNoiseLevel& level);

std::ostream& operator<<(std::ostream& os, const ImuSensorModel& model);

} // namespace invariant::test

#endif // INVARIANT_IMU_SENSOR_MODEL_H
