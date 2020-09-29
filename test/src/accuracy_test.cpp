#include "accuracy_test.h"

#include <algorithm>
#include <exception>
#include <iterator>
#include <memory>
#include <numeric>
#include <typeinfo>
#include <vector>

#include <ros/time.h>
#include <ros/duration.h>

#include <ugl/math/vector.h>
#include <ugl/math/matrix.h>
#include <ugl/math/quaternion.h>
#include <ugl/lie_group/rotation.h>
#include <ugl/lie_group/pose.h>
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

class SensorEvent
{
public:
    virtual ~SensorEvent() = default;

    const auto& time() const { return m_time; }

protected:
    SensorEvent(const ros::Time& time) : m_time(time) {}

private:
    ros::Time m_time;
};

class ImuEvent: public SensorEvent
{
public:
    ImuEvent(const ros::Time& time, double dt, ugl::Vector3 acc, ugl::Vector3 rate)
        : SensorEvent(time)
        , m_dt(dt)
        , m_acc(acc)
        , m_rate(rate)
    {
    }

    const auto& dt() { return m_dt; }
    const auto& acc() { return m_acc; }
    const auto& rate() { return m_rate; }

private:
    double m_dt;
    ugl::Vector3 m_acc;
    ugl::Vector3 m_rate;
};

class MocapEvent: public SensorEvent
{
public:
    MocapEvent(const ros::Time& time, ugl::lie::Pose pose)
        : SensorEvent(time)
        , m_pose(pose)
    {
    }

    const auto& pose() { return m_pose; }

private:
    ugl::lie::Pose m_pose;
};

using EventPtr = std::shared_ptr<SensorEvent>;

using Events = std::vector<EventPtr>;

struct Estimate
{
    ros::Time timestamp;
    ugl::lie::ExtendedPose state;

    Estimate() = default;

    Estimate(const ros::Time& t_timestamp, const ugl::lie::ExtendedPose t_state)
        : timestamp(t_timestamp)
        , state(t_state)
    {
    }
};

/// @brief Generate data-events from a virtual trajectory
/// @param imu the IMU sensor model used to generate IMU data
/// @param mocap the motion-capture sensor model used to generate motion-capture data
/// @return Vector of data-events
Events generate_events(const ugl::trajectory::Trajectory& trajectory,
                       const ImuSensorModel& imu,
                       const MocapSensorModel& mocap)
{
    Events events{};

    const ros::Time start_time{0.0};
    const ros::Time end_time{trajectory.duration()};
    const ros::Duration dt{0.001};

    const ros::Duration imu_period{imu.period()};
    const ros::Duration mocap_period{mocap.period()};
    ros::Time next_imu_time = start_time + imu_period;
    ros::Time next_mocap_time = start_time + mocap_period;

    for (ros::Time time = start_time; time <= end_time; time += dt)
    {
        if (time >= next_imu_time)
        {
            const double t = next_imu_time.toSec();
            events.push_back(std::make_shared<ImuEvent>(next_imu_time, imu.period(),
                                                        imu.get_accel_reading(t, trajectory),
                                                        imu.get_gyro_reading(t, trajectory)));
            next_imu_time += imu_period;
        }

        if (time >= next_mocap_time)
        {
            const double t = next_mocap_time.toSec();
            events.push_back(std::make_shared<MocapEvent>(next_mocap_time,
                                                          mocap.get_pose_reading(t, trajectory)));
            next_mocap_time += mocap_period;
        }
    }

    return events;
}

/// @brief Updates a filter with a sensor event
/// @param filter [in,out] the filter to update
/// @param event [in] the sensor event
template<typename FilterType>
void update_filter(FilterType& filter, const SensorEvent& event)
{
    if (typeid(event) == typeid(ImuEvent))
    {
        auto imu_event = dynamic_cast<const ImuEvent&>(event);
        filter.predict(imu_event.dt(), imu_event.acc(), imu_event.rate());
        return;
    }
    else if (typeid(event) == typeid(MocapEvent))
    {
        auto mocap_event = dynamic_cast<const MocapEvent&>(event);
        filter.mocap_update(mocap_event.pose());
        return;
    }

    throw std::logic_error("Sensor event could not be downcasted to any known sensor type.");
}

/// @brief Run filter on collection of data-events
/// @param filter the filter to use (copied before use)
/// @param events the collection of events (assumed chronologically ordered)
/// @return Vector of timestamps and estimated states
template<typename FilterType>
std::vector<Estimate> run_filter(FilterType filter, const Events& events)
{
    std::vector<Estimate> estimates{};

    const ros::Time start_time{0.0};
    const ros::Time end_time = events.back()->time();
    const ros::Duration dt{0.01};

    auto event_it = std::cbegin(events);
    for (ros::Time time = start_time; time <= end_time; time += dt)
    {
        while (event_it != std::cend(events) && (*event_it)->time() <= time)
        {
            update_filter(filter, **event_it);
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
        const ugl::Vector3 true_pos = trajectory.get_position(t);
        const ugl::Vector3 true_vel = trajectory.get_velocity(t);
        const ugl::UnitQuaternion true_quat = trajectory.get_quaternion(t);

        const ugl::Vector3 predicted_pos = estimate.state.position();
        const ugl::Vector3 predicted_vel = estimate.state.velocity();
        const ugl::UnitQuaternion predicted_quat = estimate.state.rotation().to_quaternion();

        result.position_errors.push_back(dist(true_pos, predicted_pos));
        result.velocity_errors.push_back(dist(true_vel, predicted_vel));
        result.rotation_errors.push_back(dist(true_quat, predicted_quat));
        result.times.push_back(t);
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
    filter_.set_pos(trajectory_.get_position(0.0));
    filter_.set_vel(trajectory_.get_velocity(0.0));
    filter_.set_rot(trajectory_.get_rotation(0.0));
    auto events = generate_events(trajectory_, imu_, mocap_);
    auto estimates = run_filter<invariant::IEKF>(filter_, events);
    return calculate_result(trajectory_, estimates);
}

Result MekfTestSuite::compute_accuracy()
{
    filter_.set_pos(trajectory_.get_position(0.0));
    filter_.set_vel(trajectory_.get_velocity(0.0));
    filter_.set_rot(trajectory_.get_rotation(0.0));
    auto events = generate_events(trajectory_, imu_, mocap_);
    auto estimates = run_filter<mekf::MEKF>(filter_, events);
    return calculate_result(trajectory_, estimates);
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
