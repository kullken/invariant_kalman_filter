#ifndef INVARIANT_TEST_SENSOR_EVENT_H
#define INVARIANT_TEST_SENSOR_EVENT_H

#include <string>
#include <type_traits>
#include <variant>

#include "imu_sensor_model.h"
#include "mocap_sensor_model.h"
#include "gps_sensor_model.h"

namespace invariant::test
{

class SensorEvent
{
public:
    using DataVariant = std::variant<ImuData, MocapData, GpsData>;

public:
    template<typename DataType>
    SensorEvent(double time, const DataType& data)
        : m_time(time)
        , m_data(data)
    {
    }

    auto time() const
    {
        return m_time;
    }

    const DataVariant& get_variant() const
    {
        return m_data;
    }

    template<typename FilterType>
    void update_filter(FilterType& filter) const
    {
        auto visitor = [&](auto&& data) {
            using T = std::decay_t<decltype(data)>;
            if constexpr (std::is_same_v<T, ImuData>) {
                filter.predict(data.dt, data.acc, data.rate, data.model);
            } else {
                filter.update(data.measurement, data.model);
            }
        };

        std::visit(visitor, m_data);
    }

private:
    double m_time;
    DataVariant m_data;
};

} // namespace invariant::test

#endif // INVARIANT_TEST_SENSOR_EVENT_H
