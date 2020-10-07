#include "accuracy_test.h"

#include <algorithm>
#include <iterator>
#include <numeric>
#include <vector>

#include <ros/time.h>
#include <ros/duration.h>

#include <ugl/math/vector.h>
#include <ugl/math/matrix.h>
#include <ugl/math/quaternion.h>
#include <ugl/lie_group/extended_pose.h>
#include <ugl/trajectory/trajectory.h>

#include "iekf.h"
#include "mekf.h"

#include "virtual_sensor.h"
#include "sensor_event.h"

namespace invariant::test
{

namespace
{

double dist(const ugl::Vector3& a, const ugl::Vector3& b)
{
    return (a - b).norm();
}

double dist(const ugl::UnitQuaternion& a, const ugl::UnitQuaternion& b)
{
    return a.angularDistance(b);
}

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
        const std::vector<VirtualSensor>& sensors)
{
    std::vector<SensorEvent> events{};

    const ros::Time start_time{0.0};
    const ros::Time end_time{trajectory.duration()};

    for (const auto& sensor: sensors)
    {
        const ros::Duration period{sensor.period()};
        for (ros::Time time = start_time+period; time <= end_time; time += period)
        {
            const double t = time.toSec();
            events.push_back(sensor.generate_event(t, trajectory));
        }
    }

    // Using stable sort to guarantee deterministic ordering across compiler implementations.
    std::stable_sort(std::begin(events), std::end(events), [](const SensorEvent& a, const SensorEvent& b) { return a.time() < b.time(); });
    return events;
}

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

/// @brief Compares recorded estimates with ground truth trajectory
/// @param estimates the estimates from the filter
/// @param trajectory the ground truth trajectory
/// @return Result of the comparision
Result calculate_result(const ugl::trajectory::Trajectory& trajectory, const std::vector<Estimate>& estimates)
{
    Result result{};

    for (const auto& estimate : estimates)
    {
        const double t = estimate.timestamp.toSec();
        const ugl::lie::ExtendedPose ground_truth = trajectory.get_extended_pose(t);
        const ugl::Vector3 true_pos = ground_truth.position();
        const ugl::Vector3 true_vel = ground_truth.velocity();
        const ugl::UnitQuaternion true_quat = ground_truth.rotation().to_quaternion();

        const ugl::Vector3 predicted_pos = estimate.state.position();
        const ugl::Vector3 predicted_vel = estimate.state.velocity();
        const ugl::UnitQuaternion predicted_quat = estimate.state.rotation().to_quaternion();

        result.times.push_back(t);
        result.position_errors.push_back(dist(true_pos, predicted_pos));
        result.velocity_errors.push_back(dist(true_vel, predicted_vel));
        result.rotation_errors.push_back(dist(true_quat, predicted_quat));

        result.estimates.push_back(estimate.state);
        result.ground_truth.push_back(ground_truth);
    }

    const auto count = result.times.size();
    auto square_and_add = [](double sum, double item) { return sum + item*item; };

    result.position_rmse = std::sqrt(std::accumulate(std::cbegin(result.position_errors), std::cend(result.position_errors), 0.0, square_and_add) / count);
    result.velocity_rmse = std::sqrt(std::accumulate(std::cbegin(result.velocity_errors), std::cend(result.velocity_errors), 0.0, square_and_add) / count);
    result.rotation_rmse = std::sqrt(std::accumulate(std::cbegin(result.rotation_errors), std::cend(result.rotation_errors), 0.0, square_and_add) / count);

    return result;
}

} // namespace

Result IekfTestSuite::compute_accuracy()
{
    auto events = generate_events(trajectory_, sensors_);
    auto estimates = run_filter(filter_, events);
    return calculate_result(trajectory_, estimates);
}

Result MekfTestSuite::compute_accuracy()
{
    auto events = generate_events(trajectory_, sensors_);
    auto estimates = run_filter(filter_, events);
    return calculate_result(trajectory_, estimates);
}

std::ostream& operator<<(std::ostream& os, const Result& result)
{
    constexpr auto delimiter = ' ';
    os << "time" << delimiter
       << "pos_err" << delimiter << "vel_err" << delimiter << "rot_err" << delimiter
       << "px_pred" << delimiter << "py_pred" << delimiter << "pz_pred" << delimiter
       << "vx_pred" << delimiter << "vy_pred" << delimiter << "vz_pred" << delimiter
       << "qx_pred" << delimiter << "qy_pred" << delimiter << "qz_pred" << delimiter << "qw_pred" << delimiter
       << "px_true" << delimiter << "py_true" << delimiter << "pz_true" << delimiter
       << "vx_true" << delimiter << "vy_true" << delimiter << "vz_true" << delimiter
       << "qx_true" << delimiter << "qy_true" << delimiter << "qz_true" << delimiter << "qw_true" << delimiter
       << '\n';

    auto write_vector = [&](const ugl::Vector3& vec) {
        os << vec.x() << delimiter << vec.y() << delimiter << vec.z() << delimiter;
    };
    auto write_quat = [&](const ugl::UnitQuaternion& quat) {
        os << quat.x() << delimiter << quat.y() << delimiter << quat.z() << delimiter << quat.w() << delimiter;
    };

    const auto size = result.times.size();
    for (std::size_t i = 0; i < size; ++i)
    {
        os << result.times[i] << delimiter
           << result.position_errors[i] << delimiter
           << result.velocity_errors[i] << delimiter
           << result.rotation_errors[i] << delimiter;

        write_vector(result.estimates[i].position());
        write_vector(result.estimates[i].velocity());
        write_quat(result.estimates[i].rotation().to_quaternion());

        write_vector(result.ground_truth[i].position());
        write_vector(result.ground_truth[i].velocity());
        write_quat(result.ground_truth[i].rotation().to_quaternion());

        os << '\n';
    }

    return os;
}

} // namespace invariant::test
