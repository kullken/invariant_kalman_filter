#include "accuracy_test.h"

#include <iostream>

#include <algorithm>
#include <iterator>
#include <numeric>
#include <vector>

#include <ugl/math/vector.h>
#include <ugl/math/matrix.h>
#include <ugl/math/quaternion.h>
#include <ugl/trajectory/trajectory.h>

namespace invariant::test
{

static double dist(const ugl::Vector3& a, const ugl::Vector3& b)
{
    return (a - b).norm();
}

static double dist(const ugl::UnitQuaternion& a, const ugl::UnitQuaternion& b)
{
    return a.angularDistance(b);
}

static std::vector<int> range(int start, int end, int step)
{
    int count = (end - start) / step;
    std::vector<int> values(count);
    
    double value = start - step;
    std::generate_n(std::begin(values), count, [&]{ return value += step; });

    return values;
}

AccuracyTest::Result AccuracyTest::compute_accuracy(IEKF filter, const ugl::trajectory::Trajectory &traj)
{
    AccuracyTest::Result result;

    std::vector<double> position_errors;
    std::vector<double> velocity_errors;
    std::vector<double> rotation_errors;

    // TODO: How to deal with sensors at different Hz?

    const int dt_ms = 10;
    const int duration_ms = static_cast<int>(traj.duration() * 1000.0);
    const auto times_ms = range(0, duration_ms+dt_ms, dt_ms);

    auto ms_to_sec = [](int a){ return a / 1000.0; };
    const double dt = ms_to_sec(dt_ms);
    std::vector<double> times;
    std::transform(std::cbegin(times_ms), std::cend(times_ms), std::back_inserter(times), ms_to_sec);
    
    for (const auto& t : times)
    {
        const ugl::Vector3 true_pos = traj.get_position(t);
        const ugl::Vector3 true_vel = traj.get_velocity(t);
        const ugl::Rotation true_rot = traj.get_rotation(t);

        // Update filter
        filter.predict(dt, traj.get_acceleration(t), traj.get_angular_velocity(t));
        filter.mocap_update(true_rot, true_pos);

        const ugl::Vector3 predicted_pos = filter.get_pos();
        const ugl::Vector3 predicted_vel = filter.get_vel();
        const ugl::UnitQuaternion predicted_quat = filter.get_quat();

        // Save data
        const double pos_error = dist(true_pos, predicted_pos);
        const double vel_error = dist(true_vel, predicted_vel);
        const double rot_error = dist(static_cast<ugl::UnitQuaternion>(true_rot), predicted_quat);
        position_errors.push_back(pos_error);
        velocity_errors.push_back(vel_error);
        rotation_errors.push_back(rot_error);
    }

    const auto count = times.size();
    auto square_and_add = [](double sum, double item) { return sum + item*item; };

    result.position_rmse = std::sqrt(std::accumulate(std::cbegin(position_errors), std::cend(position_errors), 0.0, square_and_add) / count);
    result.velocity_rmse = std::sqrt(std::accumulate(std::cbegin(velocity_errors), std::cend(velocity_errors), 0.0, square_and_add) / count);
    result.rotation_rmse = std::sqrt(std::accumulate(std::cbegin(rotation_errors), std::cend(rotation_errors), 0.0, square_and_add) / count);

    std::cout << "result.position_rmse : " << result.position_rmse << '\n';
    std::cout << "result.velocity_rmse : " << result.velocity_rmse << '\n';
    std::cout << "result.rotation_rmse : " << result.rotation_rmse << '\n';

    return result;
}

} // namespace invariant::test
