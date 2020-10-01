#ifndef INVARIANT_TEST_SENSOR_MODEL_H
#define INVARIANT_TEST_SENSOR_MODEL_H

#include <variant>

#include <ugl/trajectory/trajectory.h>

#include "sensor_event.h"
#include "imu_sensor_model.h"
#include "mocap_sensor_model.h"

namespace invariant::test
{

class VirtualSensor
{
public:
    template<typename SensorType>
    VirtualSensor(const SensorType& sensor)
        : m_sensor(sensor)
    {
    }

    double period() const;

    SensorEvent generate_event(double t, const ugl::trajectory::Trajectory& trajectory) const;

private:
    std::variant<ImuSensorModel, MocapSensorModel> m_sensor;
};

} // namespace invariant::test

#endif // INVARIANT_TEST_SENSOR_MODEL_H
