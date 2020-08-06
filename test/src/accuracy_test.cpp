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

}

template<typename FilterType>
Result AccuracyTest<FilterType>::compute_accuracy_impl()
{
    Result result;

    const double measurement_period = 0.01;

    const auto clock = create_clock(trajectory.duration());

    double next_imu_time = clock[0] + imu_.period();
    double next_mocap_time = clock[0] + mocap_.period();
    double next_measurement_time = clock[0];

    filter_.set_pos(trajectory_.get_position(clock[0]));
    filter_.set_vel(trajectory_.get_velocity(clock[0]));
    filter_.set_rot(trajectory_.get_rotation(clock[0]));
    
    for (const auto& t : clock)
    {
        if (t >= next_imu_time)
        {
            filter_.predict(imu_.period(), imu_.get_accel_reading(next_imu_time), imu_.get_gyro_reading(next_imu_time));
            next_imu_time += imu_.period();
        }

        if (t >= next_mocap_time)
        {
            filter_.mocap_update(mocap_.get_rot_reading(next_mocap_time), mocap_.get_pos_reading(next_mocap_time));
            next_mocap_time += mocap_.period();
        }

        if (t >= next_measurement_time)
        {
            const ugl::Vector3 true_pos = trajectory_.get_position(next_measurement_time);
            const ugl::Vector3 true_vel = trajectory_.get_velocity(next_measurement_time);
            const ugl::UnitQuaternion true_quat = trajectory_.get_quaternion(next_measurement_time);

            const ugl::Vector3 predicted_pos = filter_.get_pos();
            const ugl::Vector3 predicted_vel = filter_.get_vel();
            const ugl::UnitQuaternion predicted_quat = filter_.get_quat();

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

Result IekfTestSuite::compute_accuracy()
{
    return compute_accuracy_impl();
}

Result MekfTestSuite::compute_accuracy()
{
    return compute_accuracy_impl();
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
