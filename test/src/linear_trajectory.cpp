#include "linear_trajectory.h"

#include <vector>
#include <algorithm>

#include "iekf_types.h"

namespace invariant::test
{

LinearTrajectory::LinearTrajectory(std::vector<Segment> segments)
    : m_segments(segments)
{
    for (auto& segment : m_segments)
    {
        segment.time_offset = m_duration;
        m_duration += segment.duration();
    }
    // TODO: Assert that all segments are continously connected up to the x(?):th derivative.
}

Vector3 LinearTrajectory::get_position(double t) const
{
    const Segment& segment = get_segment_at(t);
    return segment.pos(t - segment.time_offset);
}

Vector3 LinearTrajectory::get_velocity(double t) const
{
    const Segment& segment = get_segment_at(t);
    return segment.vel(t - segment.time_offset);
}

Vector3 LinearTrajectory::get_acceleration(double t) const
{
    const Segment& segment = get_segment_at(t);
    return segment.acc(t - segment.time_offset);
}

const LinearTrajectory::Segment& LinearTrajectory::get_segment_at(double t) const
{
    // TODO: Assert: 0 <= t <= m_duration

    for (auto it = std::rbegin(m_segments); it != std::rend(m_segments); ++it)
    {
        if (it->time_offset < t)
        {
            return *it;
        }
    }

    // NOTE: This should be unreachable. What to do? Assert(false)?
}

}