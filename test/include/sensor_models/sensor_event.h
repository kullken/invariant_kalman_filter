#ifndef INVARIANT_TEST_SENSOR_EVENT_H
#define INVARIANT_TEST_SENSOR_EVENT_H

#include <type_traits>
#include <variant>

#include "imu_sensor_model.h"
#include "mocap_sensor_model.h"

namespace invariant::test
{

class SensorEvent
{
public:
    template<typename DataType>
    SensorEvent(double time, const DataType& data)
        : m_time(time)
        , m_data(data)
    {
    }

    auto time() const { return m_time; }

    template<typename FilterType>
    void update_filter(FilterType& filter) const
    {
        auto visitor = [&](auto&& data) {
            using T = std::decay_t<decltype(data)>;
            if constexpr (std::is_same_v<T, ImuData>)
                filter.predict(data.dt, data.acc, data.rate);
            else if constexpr (std::is_same_v<T, MocapData>)
                filter.mocap_update(data.pose);
            else
                static_assert(always_false_v<T>, "Visitor does not handle all sensor types!");
        };

        std::visit(visitor, m_data);
    }

private:
    double m_time;
    std::variant<ImuData, MocapData> m_data;

    template<typename>
    [[maybe_unused]] inline static
    constexpr bool always_false_v = false;
};

} // namespace invariant::test

#endif // INVARIANT_TEST_SENSOR_EVENT_H