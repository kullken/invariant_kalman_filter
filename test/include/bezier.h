#pragma once

#include <array>
#include <cmath>
#include <utility>

namespace trajectory
{

static constexpr unsigned int factorial(unsigned int n)
{
    return (n == 1 || n == 0) ? 1 : n * factorial(n - 1);
}

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

private:
    PointType calc_value(double t) const
    {
        // TODO: Use De Casteljau's algorithm for better numerical stability at higher degrees?
        PointType result{};
        for (const auto& coeff : calc_coeffs())
        {
            result = result*t + coeff;
        }
        return result;
    }

    std::array<PointType, size> calc_coeffs() const
    {
        std::array<PointType, size> coeffs;
        for (int j = 0; j <= degree; ++j)
        {
            PointType cj{};
            for (int i = 0; i <= j; ++i)
            {
                cj += std::pow(-1, i+j) * points_[i] / (factorial(i) * factorial(j-i));
            }
            cj *= factorial(degree) / factorial(degree - j);
            cj /= std::pow(T_, j);
            coeffs[j] = cj;
        }
        return std::reverse(coeffs);
    }

};

/// Split Bézier-curve into two new curves at time t.
template<typename PointType, std::size_t degree>
std::pair<Bezier<PointType, degree>, Bezier<PointType, degree>> split(Bezier<PointType, degree> bezier, double t);

}
