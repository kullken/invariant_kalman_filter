#include "accuracy_test.h"

#include <algorithm>
#include <iterator>
#include <numeric>
#include <vector>

#include <ros/time.h>
#include <ros/duration.h>

#include <ugl/math/vector.h>
#include <ugl/math/matrix.h>
#include <ugl/lie_group/extended_pose.h>
#include <ugl/trajectory/trajectory.h>

#include "virtual_sensor.h"
#include "sensor_event.h"

namespace invariant::test
{

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
        const auto ground_truth = trajectory.get_extended_pose(t);
        const auto true_pos = ground_truth.position();
        const auto true_vel = ground_truth.velocity();
        const auto true_rot = ground_truth.rotation();

        const auto predicted_pos = estimate.state.position();
        const auto predicted_vel = estimate.state.velocity();
        const auto predicted_rot = estimate.state.rotation();

        result.times.push_back(t);
        result.position_errors.emplace_back(true_pos - predicted_pos);
        result.velocity_errors.emplace_back(true_vel - predicted_vel);
        result.rotation_errors.emplace_back(ominus(true_rot, predicted_rot));

        result.estimates.push_back(estimate.state);
        result.ground_truth.push_back(ground_truth);

        const ugl::Vector<9> log_error = ominus(ground_truth, estimate.state);
        const double nees = log_error.transpose() * estimate.covariance.inverse() * log_error;
        result.nees_values.push_back(nees);
        result.nis_values.push_back(estimate.nis);

        const ugl::Vector<3> pos_log_error = log_error.segment<3>(6);
        const ugl::Vector<3> vel_log_error = log_error.segment<3>(3);
        const ugl::Vector<3> rot_log_error = log_error.segment<3>(0);

        const double pos_nees = pos_log_error.transpose() * estimate.covariance.block<3,3>(6,6).inverse() * pos_log_error;
        const double vel_nees = vel_log_error.transpose() * estimate.covariance.block<3,3>(3,3).inverse() * vel_log_error;
        const double rot_nees = rot_log_error.transpose() * estimate.covariance.block<3,3>(0,0).inverse() * rot_log_error;

        result.position_nees.push_back(pos_nees);
        result.velocity_nees.push_back(vel_nees);
        result.rotation_nees.push_back(rot_nees);
    }

    const auto count = result.times.size();
    auto square_and_add = [](double sum, const ugl::Vector3& item) { return sum + item.squaredNorm(); };

    result.position_rmse = std::sqrt(std::accumulate(std::cbegin(result.position_errors), std::cend(result.position_errors), 0.0, square_and_add) / count);
    result.velocity_rmse = std::sqrt(std::accumulate(std::cbegin(result.velocity_errors), std::cend(result.velocity_errors), 0.0, square_and_add) / count);
    result.rotation_rmse = std::sqrt(std::accumulate(std::cbegin(result.rotation_errors), std::cend(result.rotation_errors), 0.0, square_and_add) / count);

    return result;
}

} // namespace invariant::test
