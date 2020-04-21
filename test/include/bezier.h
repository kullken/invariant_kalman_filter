#pragma once

#include <array>
#include <utility>

namespace trajectory
{

template<typename PointType, std::size_t degree>
class Bezier
{
    static constexpr std::size_t size = degree + 1;

private:
    const double T_ = 1;
    const std::array<PointType, size> points_;

public:
    explicit Bezier(std::array<PointType, size> points) : points_(points)
    {
    }

    Bezier(double duration, std::array<PointType, size> points)
        : T_(duration)
        , points_(points)
    {
        // TODO: Assert: T_ > 0
    }

    PointType pos(double t) const { return calc_value(t); }
    PointType vel(double t) const { return get_derivative().pos(t); }
    PointType acc(double t) const { return get_derivative().vel(t); }

    Bezier<PointType, degree-1> get_derivative() const;
    Bezier<PointType, degree> get_reversed() const;

};

/// Split Bézier-curve into two new curves at time t.
template<typename PointType, std::size_t degree>
std::pair<Bezier<PointType, degree>, Bezier<PointType, degree>> split(Bezier<PointType, degree> bezier, double t);

}
