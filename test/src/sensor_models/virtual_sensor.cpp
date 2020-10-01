#include "virtual_sensor.h"

#include <variant>

#include <ugl/trajectory/trajectory.h>

#include "sensor_event.h"
#include "imu_sensor_model.h"
#include "mocap_sensor_model.h"

namespace invariant::test
{

double VirtualSensor::period() const
{
    auto visitor = [&](const auto& sensor) {
        return sensor.period();
    };
    return std::visit(visitor, m_sensor);
}

SensorEvent VirtualSensor::generate_event(double t, const ugl::trajectory::Trajectory& trajectory) const
{
    auto visitor = [&](const auto& sensor) {
        return SensorEvent{t, sensor.get_data(t, trajectory)};
    };
    return std::visit(visitor, m_sensor);
}

} // namespace invariant::test
