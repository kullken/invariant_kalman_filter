#include "gps_sensor_model.h"

#include <exception>
#include <ostream>

#include <ugl/math/vector.h>
#include <ugl/math/matrix.h>
#include <ugl/lie_group/euclidean.h>
#include <ugl/trajectory/trajectory.h>

namespace invariant::test
{

static ugl::Matrix3 get_position_covar(GpsNoiseLevel level);

GpsSensorModel::GpsSensorModel(GpsNoiseLevel level, double frequency, const GpsModel::MeasurementType& offset)
    : m_noise_level(level)
    , m_period(1.0/frequency)
    , m_position_noise(get_position_covar(level))
    , m_model(offset, level == GpsNoiseLevel::None ? get_position_covar(GpsNoiseLevel::Low) : get_position_covar(level))
{
}

GpsSensorModel::GpsSensorModel(const ugl::Matrix3& covariance, double frequency, const GpsModel::MeasurementType& offset)
    : GpsSensorModel(covariance, covariance, frequency, offset)
{
}

GpsSensorModel::GpsSensorModel(
    const ugl::Matrix3& true_covariance,
    const ugl::Matrix3& believed_covariance,
    double frequency,
    const GpsModel::MeasurementType& offset
)
    : m_noise_level(GpsNoiseLevel::Custom)
    , m_period(1.0/frequency)
    , m_position_noise(true_covariance)
    , m_model(offset, believed_covariance)
{
}

GpsData GpsSensorModel::get_data(double t, const ugl::trajectory::Trajectory& trajectory) const
{
    return GpsData{ugl::lie::Euclidean<3>{get_pos_reading(t, trajectory)}, m_model};
}

ugl::Vector3 GpsSensorModel::get_pos_reading(double t, const ugl::trajectory::Trajectory& trajectory) const
{
    return m_model.h(trajectory.get_extended_pose(t), m_position_noise.sample());
}

std::ostream& operator<<(std::ostream& os, const GpsNoiseLevel& level)
{
    switch (level)
    {
    case GpsNoiseLevel::None:
        return os << "None";
    case GpsNoiseLevel::Low:
        return os << "Low";
    case GpsNoiseLevel::High:
        return os << "High";
    case GpsNoiseLevel::Custom:
        return os << "Custom";
    }
}

std::ostream& operator<<(std::ostream& os, const GpsSensorModel& model)
{
    return os << "Gps Noise: " << model.noise_level();
}

template<GpsNoiseLevel level>
static ugl::Matrix3 position_covar();

template<>
ugl::Matrix3 position_covar<GpsNoiseLevel::None>()
{
    return ugl::Matrix3::Zero();
}

template<>
ugl::Matrix3 position_covar<GpsNoiseLevel::Low>()
{
    constexpr double stddev = 0.1;  // [m]
    constexpr double variance = stddev*stddev;
    return ugl::Matrix3::Identity() * variance;
}

template<>
ugl::Matrix3 position_covar<GpsNoiseLevel::High>()
{
    constexpr double stddev = 1;  // [m]
    constexpr double variance = stddev*stddev;
    return ugl::Matrix3::Identity() * variance;
}

static ugl::Matrix3 get_position_covar(GpsNoiseLevel level)
{
    switch (level)
    {
    case GpsNoiseLevel::None:
        return position_covar<GpsNoiseLevel::None>();
    case GpsNoiseLevel::Low:
        return position_covar<GpsNoiseLevel::Low>();
    case GpsNoiseLevel::High:
        return position_covar<GpsNoiseLevel::High>();
    default:
        throw std::logic_error("The asked for GpsNoiseLevel is not yet implemented.");
    }
}

} // namespace invariant::test
