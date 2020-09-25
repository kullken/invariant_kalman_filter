#ifndef INVARIANT_IMU_SENSOR_MODEL_H
#define INVARIANT_IMU_SENSOR_MODEL_H

#include <ostream>

#include <ugl/math/vector.h>
#include <ugl/trajectory/trajectory.h>
#include <ugl/random/normal_distribution.h>

namespace invariant::test
{

enum class ImuNoiseLevel
{
    None,
    Low,
    High,
    Mueller18,
};

/// A class for representing virtual IMU sensors.
/// TODO: Add bias?.
/// TODO: Enable arbitrary IMU coordinate system.
class ImuSensorModel
{
public:
    ImuSensorModel() : ImuSensorModel(ImuNoiseLevel::None) {}

    explicit ImuSensorModel(ImuNoiseLevel level, double frequency=100.0);

    ImuSensorModel(ImuNoiseLevel level, double frequency, const ugl::trajectory::Trajectory& trajectory);

    double period() const
    {
        return period_;
    }

    ImuNoiseLevel noise_level() const
    {
        return noise_level_;
    }

    void set_trajectory(const ugl::trajectory::Trajectory& trajectory)
    {
        trajectory_ = trajectory;
    }

    /// @return An accelerometer reading expressed in the body frame.
    ugl::Vector3 get_accel_reading(double t, const ugl::trajectory::Trajectory& trajectory) const;

    /// @return An gyroscope reading expressed in the body frame.
    ugl::Vector3 get_gyro_reading(double t, const ugl::trajectory::Trajectory& trajectory) const;

private:
    ImuNoiseLevel noise_level_;
    double period_;
    ugl::trajectory::Trajectory trajectory_;

    ugl::random::NormalDistribution<3> accel_noise_;
    ugl::random::NormalDistribution<3> gyro_noise_;

    static const ugl::Vector3 s_gravity;
};

inline std::ostream& operator<<(std::ostream& os, const ImuNoiseLevel& level)
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

inline std::ostream& operator<<(std::ostream& os, const ImuSensorModel& model)
{
    return os << "IMU Noise: " << model.noise_level();
}

} // namespace invariant::test

#endif // INVARIANT_IMU_SENSOR_MODEL_H
