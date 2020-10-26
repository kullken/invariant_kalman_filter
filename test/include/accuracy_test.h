#ifndef INVARIANT_ACCURACY_TEST_H
#define INVARIANT_ACCURACY_TEST_H

#include <tuple>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <ros/time.h>
#include <ros/duration.h>

#include <ugl/lie_group/extended_pose.h>
#include <ugl/trajectory/trajectory.h>
#include <ugl/random/seed.h>

#include "iekf.h"
#include "mekf.h"

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
    std::vector<double> position_errors;
    std::vector<double> velocity_errors;
    std::vector<double> rotation_errors;

    std::vector<ugl::lie::ExtendedPose> estimates;
    std::vector<ugl::lie::ExtendedPose> ground_truth;
};

struct Estimate
{
    ros::Time timestamp;
    ugl::lie::ExtendedPose state;

    Estimate() = default;

    Estimate(const ros::Time& t_timestamp, const ugl::lie::ExtendedPose& t_state)
        : timestamp(t_timestamp)
        , state(t_state)
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
        while (event_it != std::cend(events) && event_it->time() <= time.toSec())
        {
            event_it->update_filter(filter);
            ++event_it;
        }
        estimates.emplace_back(time, filter.get_state());
    }

    return estimates;
}

template<typename FilterType>
class AccuracyTest : public testing::TestWithParam<std::tuple<TestTrajectory, std::vector<VirtualSensor>>>
{
protected:
    AccuracyTest()
        : trajectory_(std::get<0>(GetParam()).traj)
        , sensors_(std::get<1>(GetParam()))
    {
        ugl::random::set_seed(117);
    }

    Result compute_accuracy(const std::vector<SensorEvent>& events)
    {
        const auto initial_error = offset_.sample_uniform();
        const auto initial_state = trajectory_.get_extended_pose(0.0) * initial_error;
        filter_.set_state(initial_state);

        auto estimates = run_filter(filter_, events);
        return calculate_result(trajectory_, estimates);
    }

protected:
    FilterType filter_{};
    OffsetGenerator offset_{};
    ugl::trajectory::Trajectory trajectory_;
    std::vector<VirtualSensor> sensors_;
};

} // namespace invariant::test

#endif // INVARIANT_ACCURACY_TEST_H
