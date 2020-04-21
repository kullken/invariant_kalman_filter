#include "linear_trajectory.h"

#include <vector>

#include "iekf_types.h"

namespace invariant::test
{

LinearTrajectory::LinearTrajectory(std::vector<LinearTrajectorySegment> segments)
    : m_segments(segments)
{
    
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