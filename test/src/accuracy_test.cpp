#include "accuracy_test.h"

#include <algorithm>
#include <vector>

#include <ugl/trajectory/trajectory.h>

namespace invariant::test
{

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
        filter.predict(dt, traj.get_acceleration(t), traj.get_angular_velocity(t));
        filter.mocap_update(traj.get_rotation(t), traj.get_position(t));

        // Calculate/update error
        // TODO: Figure out what to measure.
        // TODO: Update result variable.
    }
    return result;
}

} // namespace invariant::test
