#ifndef INVARIANT_TEST_TRAJECTORIES_H
#define INVARIANT_TEST_TRAJECTORIES_H

#include <iostream>
#include <string>

#include <ugl/math/vector.h>
#include <ugl/trajectory/trajectory.h>


namespace invariant::test
{

struct TestTrajectory
{
    std::string description;
    ugl::trajectory::Trajectory traj;
};

inline std::ostream& operator<<(std::ostream& os, const TestTrajectory& param)
{
    return os << param.description;
}

ugl::trajectory::Trajectory stand_still(double duration=1.0);

ugl::trajectory::Trajectory rotate_in_place(double degrees, double duration=1.0);

ugl::trajectory::Trajectory constant_velocity(ugl::Vector3 velocity, double duration=1.0);

ugl::trajectory::Trajectory quadratic_translation(ugl::Vector3 delta, double duration=1.0);

/// Start and stop maneuver with constant acceleration like a spacecraft with infinite fuel.
ugl::trajectory::Trajectory start_stop(ugl::Vector3 acceleration, double duration=1.0);

} // namespace invariant::test

#endif // INVARIANT_TEST_TRAJECTORIES_H
