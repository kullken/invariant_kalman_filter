#ifndef INVARIANT_GPS_SENSOR_MODEL_H
#define INVARIANT_GPS_SENSOR_MODEL_H

#include <iosfwd>

#include <ugl/math/vector.h>
#include <ugl/lie_group/euclidean.h>
#include <ugl/trajectory/trajectory.h>
#include <ugl/random/normal_distribution.h>

#include "gps_model.h"

namespace invariant::test
{

struct GpsData
{
    ugl::lie::Euclidean<3> measurement;
    GpsModel model;
};

enum class GpsNoiseLevel
{
    None,
    Low,
    High
};

/// A class for representing virtual Motion-capture sensors.
/// TODO: Add bias.
class GpsSensorModel
{
public:
    GpsSensorModel(GpsNoiseLevel level, double frequency, const GpsModel::MeasurementType& offset=GpsModel::MeasurementType::Identity());

    double period() const
    {
        return period_;
    }

    GpsNoiseLevel noise_level() const
    {
        return noise_level_;
    }

    /// @brief Generate sensor data from a trajectory
    /// @param t the time at which to generate data
    /// @param trajectory the trajectory from which to generate data
    /// @return The sensor data
    GpsData get_data(double t, const ugl::trajectory::Trajectory& trajectory) const;

    /// @return A position reading expressed in the inertial frame.
    ugl::Vector3 get_pos_reading(double t, const ugl::trajectory::Trajectory& trajectory) const;

private:
    GpsNoiseLevel noise_level_ = GpsNoiseLevel::None;
    double period_ = 0.1;

    ugl::random::NormalDistribution<3> position_noise_;
    GpsModel model_;
};

std::ostream& operator<<(std::ostream& os, const GpsNoiseLevel& level);

std::ostream& operator<<(std::ostream& os, const GpsSensorModel& model);

} // namespace invariant::test

#endif // INVARIANT_GPS_SENSOR_MODEL_H
