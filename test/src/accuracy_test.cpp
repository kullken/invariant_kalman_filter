#include "accuracy_test.h"

#include <algorithm>
#include <iterator>
#include <numeric>
#include <vector>

#include <ugl/math/vector.h>
#include <ugl/math/matrix.h>
#include <ugl/math/quaternion.h>
#include <ugl/trajectory/trajectory.h>

#include "imu_sensor_model.h"
#include "mocap_sensor_model.h"

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

    // TODO: How to deal with sensors at different Hz?

    ImuSensorModel imu{traj};
    MocapSensorModel mocap{traj};

    const int dt_ms = 10;
    const int duration_ms = static_cast<int>(traj.duration() * 1000.0);
    const auto times_ms = range(0, duration_ms+dt_ms, dt_ms);

    auto ms_to_sec = [](int a){ return a / 1000.0; };
    const double dt = ms_to_sec(dt_ms);
    std::vector<double> times;
    std::transform(std::cbegin(times_ms), std::cend(times_ms), std::back_inserter(times), ms_to_sec);
    
    for (const auto& t : times)
    {
        // Update filter
        filter.predict(dt, imu.getAccReading(t), imu.getGyroReading(t));
        filter.mocap_update(mocap.getRotReading(t), mocap.getPosReading(t));

        const ugl::Vector3 true_pos = traj.get_position(t);
        const ugl::Vector3 true_vel = traj.get_velocity(t);
        const ugl::UnitQuaternion true_quat = static_cast<ugl::UnitQuaternion>(traj.get_rotation(t));

        const ugl::Vector3 predicted_pos = filter.get_pos();
        const ugl::Vector3 predicted_vel = filter.get_vel();
        const ugl::UnitQuaternion predicted_quat = filter.get_quat();

        // Save data
        const double pos_error = dist(true_pos, predicted_pos);
        const double vel_error = dist(true_vel, predicted_vel);
        const double rot_error = dist(true_quat, predicted_quat);
        result.position_errors.push_back(pos_error);
        result.velocity_errors.push_back(vel_error);
        result.rotation_errors.push_back(rot_error);
    }

    const auto count = times.size();
    auto square_and_add = [](double sum, double item) { return sum + item*item; };

    result.position_rmse = std::sqrt(std::accumulate(std::cbegin(result.position_errors), std::cend(result.position_errors), 0.0, square_and_add) / count);
    result.velocity_rmse = std::sqrt(std::accumulate(std::cbegin(result.velocity_errors), std::cend(result.velocity_errors), 0.0, square_and_add) / count);
    result.rotation_rmse = std::sqrt(std::accumulate(std::cbegin(result.rotation_errors), std::cend(result.rotation_errors), 0.0, square_and_add) / count);

    return result;
}

std::ostream& operator<<(std::ostream& os, const AccuracyTest::Result& result)
{
    constexpr auto delimiter = ' ';
    os << "pos_error" << delimiter << "vel_error" << delimiter << "rot_error" << '\n';

    auto size = std::min({result.position_errors.size(), result.velocity_errors.size(), result.rotation_errors.size()});
    for (std::size_t i = 0; i < size; ++i)
    {
        os << result.position_errors[i] << delimiter << result.velocity_errors[i] << delimiter << result.rotation_errors[i] << '\n';
    }

    return os;
}

} // namespace invariant::test
