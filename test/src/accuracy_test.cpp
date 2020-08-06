#include "accuracy_test.h"

#include <algorithm>
#include <iterator>
#include <numeric>
#include <vector>

#include <ugl/math/vector.h>
#include <ugl/math/matrix.h>
#include <ugl/math/quaternion.h>
#include <ugl/trajectory/trajectory.h>

#include "iekf.h"
#include "mekf.h"

#include "imu_sensor_model.h"
#include "mocap_sensor_model.h"

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

std::vector<int> range(int start, int end, int step)
{
    int count = (end - start) / step;
    std::vector<int> values(count);
    
    double value = start - step;
    std::generate_n(std::begin(values), count, [&]{ return value += step; });

    return values;
}

std::vector<double> create_clock(double duration)
{
    constexpr int dt_ms = 1;
    const int duration_ms = static_cast<int>(duration * 1000.0);
    const auto clock_ms = range(0, duration_ms+dt_ms, dt_ms);

    std::vector<double> clock(clock_ms.size());
    std::transform(std::cbegin(clock_ms), std::cend(clock_ms), std::back_inserter(clock), [](int ms){ return ms / 1000.0; } );

    return clock;
}

template<typename FilterType>
Result compute_accuracy_impl(FilterType filter, const ugl::trajectory::Trajectory &trajectory, const ImuSensorModel &imu, const MocapSensorModel &mocap)
{
    Result result;

    const auto clock = create_clock(trajectory.duration());

    double next_imu_time = clock[0] + imu.period();
    double next_mocap_time = clock[0] + mocap.period();
    double next_measurement_time = clock[0];

    filter.set_pos(trajectory.get_position(clock[0]));
    filter.set_vel(trajectory.get_velocity(clock[0]));
    filter.set_rot(trajectory.get_rotation(clock[0]));

    const double measurement_period = 0.01;

    for (const auto& t : clock)
    {
        if (t >= next_imu_time)
        {
            filter.predict(imu.period(), imu.get_accel_reading(next_imu_time), imu.get_gyro_reading(next_imu_time));
            next_imu_time += imu.period();
        }

        if (t >= next_mocap_time)
        {
            filter.mocap_update(mocap.get_rot_reading(next_mocap_time), mocap.get_pos_reading(next_mocap_time));
            next_mocap_time += mocap.period();
        }

        if (t >= next_measurement_time)
        {
            const ugl::Vector3 true_pos = trajectory.get_position(next_measurement_time);
            const ugl::Vector3 true_vel = trajectory.get_velocity(next_measurement_time);
            const ugl::UnitQuaternion true_quat = trajectory.get_quaternion(next_measurement_time);

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
            result.times.push_back(next_measurement_time);

            next_measurement_time += measurement_period;
        }
    }

    const auto count = result.times.size();
    auto square_and_add = [](double sum, double item) { return sum + item*item; };

    result.position_rmse = std::sqrt(std::accumulate(std::cbegin(result.position_errors), std::cend(result.position_errors), 0.0, square_and_add) / count);
    result.velocity_rmse = std::sqrt(std::accumulate(std::cbegin(result.velocity_errors), std::cend(result.velocity_errors), 0.0, square_and_add) / count);
    result.rotation_rmse = std::sqrt(std::accumulate(std::cbegin(result.rotation_errors), std::cend(result.rotation_errors), 0.0, square_and_add) / count);

    return result;
}

}

Result IekfTestSuite::compute_accuracy()
{
    return compute_accuracy_impl<invariant::IEKF>(filter_, trajectory_, imu_, mocap_);
}

Result MekfTestSuite::compute_accuracy()
{
    return compute_accuracy_impl<mekf::MEKF>(filter_, trajectory_, imu_, mocap_);
}

std::ostream& operator<<(std::ostream& os, const Result& result)
{
    constexpr auto delimiter = ' ';
    os << "time" << delimiter << "pos_error" << delimiter << "vel_error" << delimiter << "rot_error" << '\n';

    auto size = std::min({result.position_errors.size(), result.velocity_errors.size(), result.rotation_errors.size()});
    for (std::size_t i = 0; i < size; ++i)
    {
        os << result.times[i] << delimiter << result.position_errors[i] << delimiter << result.velocity_errors[i] << delimiter << result.rotation_errors[i] << '\n';
    }

    return os;
}

} // namespace invariant::test
