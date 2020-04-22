#pragma once

#include <vector>

#include "iekf_types.h"
#include "bezier.h"

namespace invariant::test
{

class LinearTrajectory
{
    class Segment : public trajectory::Bezier<Vector3, 3>
    {
    public:
        double time_offset;
    };

private:
    double m_duration = 0;

    std::vector<Segment> m_segments;

public:
    LinearTrajectory() = default;
    LinearTrajectory(std::vector<Segment> segments);

    Vector3 get_position(double t) const;
    Vector3 get_velocity(double t) const;
    Vector3 get_acceleration(double t) const;

private:
    const Segment& get_segment_at(double t) const;
};

}