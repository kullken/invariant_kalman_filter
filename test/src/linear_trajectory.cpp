#include "linear_trajectory.h"

#include <vector>

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

}

Vector3 LinearTrajectory::get_velocity(double t) const
{

}

Vector3 LinearTrajectory::get_acceleration(double t) const
{

}

}