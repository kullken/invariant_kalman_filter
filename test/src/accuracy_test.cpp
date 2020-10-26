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

} // namespace

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

} // namespace invariant::test
