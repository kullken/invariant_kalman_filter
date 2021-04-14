#ifndef INVARIANT_ACCURACY_TEST_H
#define INVARIANT_ACCURACY_TEST_H

#include <tuple>
#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <ros/time.h>
#include <ros/duration.h>

#include <ugl/math/matrix.h>
#include <ugl/lie_group/extended_pose.h>
#include <ugl/trajectory/trajectory.h>
#include <ugl/random/seed.h>

#include "test_trajectories.h"
#include "offset_generator.h"

#include "virtual_sensor.h"
#include "sensor_event.h"

namespace invariant::test
{

struct Result
{
    double position_rmse = 0;
    double velocity_rmse = 0;
    double rotation_rmse = 0;

    std::vector<double> times;
    std::vector<double> nees_values; // NEES - Normalized Estimation Error Squared
    std::vector<double> nis_values;  // NIS  - Normalized Innovation Squared

    std::vector<ugl::Vector3> position_errors;
    std::vector<ugl::Vector3> velocity_errors;
    std::vector<ugl::Vector3> rotation_errors;

    std::vector<ugl::lie::ExtendedPose> estimates;
    std::vector<ugl::lie::ExtendedPose> ground_truth;
};

struct Estimate
{
    ros::Time timestamp;
    ugl::lie::ExtendedPose state;
    ugl::Matrix<9,9> covariance;
    double nis;  // NIS - Normalized Innovation Squared

    Estimate() = default;

    Estimate(const ros::Time& t_timestamp,
             const ugl::lie::ExtendedPose& t_state,
             const ugl::Matrix<9,9>& t_covariance,
             double t_nis)
        : timestamp(t_timestamp)
        , state(t_state)
        , covariance(t_covariance)
        , nis(t_nis)
    {
    }
};

/// @brief Generate sensor events from a virtual trajectory
/// @param trajectory the ground truth trajectory
/// @param sensors vector of virtual sensors used to generate events
/// @return A sorted vector of sensor events
std::vector<SensorEvent> generate_events(
        const ugl::trajectory::Trajectory& trajectory,
        const std::vector<VirtualSensor>& sensors);

/// @brief Compares recorded estimates with ground truth trajectory
/// @param estimates the estimates from the filter
/// @param trajectory the ground truth trajectory
/// @return Result of the comparision
Result calculate_result(const ugl::trajectory::Trajectory& trajectory, const std::vector<Estimate>& estimates);

/// @brief Run filter on collection of data-events
/// @param filter the filter to use (copied before use)
/// @param events the collection of events (assumed chronologically ordered)
/// @return Vector of timestamps and estimated states
template<typename FilterType>
std::vector<Estimate> run_filter(FilterType filter, const std::vector<SensorEvent>& events)
{
    std::vector<Estimate> estimates{};

    const ros::Time start_time{0.0};
    const ros::Time end_time{events.back().time()};
    const ros::Duration dt{0.01};

    auto event_it = std::cbegin(events);
    for (ros::Time time = start_time; time <= end_time; time += dt)
    {
        // TODO: Currently, only the NIS-value from the last
        // measurement update of every timestep is saved.
        std::optional<double> nis{};
        while (event_it != std::cend(events) && event_it->time() <= time.toSec())
        {
            auto update_filter = [&](auto&& data) {
                using T = std::decay_t<decltype(data)>;
                if constexpr (std::is_same_v<T, ImuData>) {
                    filter.predict(data.dt, data.acc, data.rate, data.model);
                } else {
                    nis = filter.update(data.measurement, data.model);
                }
            };
            std::visit(update_filter, event_it->get_variant());
            ++event_it;
        }
        estimates.emplace_back(time, filter.get_state(), filter.get_covariance(), nis.value_or(-1));
    }

    return estimates;
}

class AccuracyTest : public testing::TestWithParam<std::tuple<TestTrajectory, std::vector<VirtualSensor>, OffsetGenerator>>
{
protected:
    AccuracyTest()
        : m_trajectory(std::get<0>(GetParam()).traj)
        , m_sensors(std::get<1>(GetParam()))
        , m_offset(std::get<2>(GetParam()))
    {
        ugl::random::set_seed(117);
    }

    template<typename FilterType>
    Result compute_accuracy(const std::vector<SensorEvent>& events)
    {
        const auto initial_error = m_offset.sample_uniform();
        const auto initial_state = m_trajectory.get_extended_pose(0.0) * initial_error;
        const auto initial_covar = m_offset.get_covariance();

        FilterType filter{initial_state, initial_covar};

        const auto estimates = run_filter(filter, events);
        return calculate_result(m_trajectory, estimates);
    }

protected:
    ugl::trajectory::Trajectory m_trajectory;
    std::vector<VirtualSensor> m_sensors;
    OffsetGenerator m_offset;
};

} // namespace invariant::test

#endif // INVARIANT_ACCURACY_TEST_H
