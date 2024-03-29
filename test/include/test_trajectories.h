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

    static TestTrajectory stand_still(double duration=1.0);
    static TestTrajectory rotate_in_place(double degrees, double duration=1.0);
    static TestTrajectory constant_velocity(ugl::Vector3 velocity, double duration=1.0);
    static TestTrajectory quadratic_translation(ugl::Vector3 delta, double duration=1.0);
    static TestTrajectory start_stop(ugl::Vector3 acceleration, double duration=1.0);

    /// @brief Circular motion in the xy-plane around the origin, oriented in direction of movement.
    /// @param degrees total rotation around the origin in degrees
    /// @param radius radius of the circle
    /// @param duration duration of the trajectory
    /// @return The generated trajectory.
    static TestTrajectory circle(double degrees, double radius, double duration);

    /// @brief Helix trajectory around the z-axis, facing in xy-direction of movement.
    /// @param degrees total rotation around the origin in degrees
    /// @param radius radius of the circle
    /// @param z_velocity velocity along the z-axis
    /// @param duration duration of the trajectory
    /// @return The generated trajectory.
    static TestTrajectory helix(double degrees, double radius, double z_velocity, double duration);

    /// @brief Hexagon shaped trajectory coming to complete stop in every corner.
    /// @param laps total number of laps around the hexagon
    /// @param duration duration of the trajectory
    /// @return The generated trajectory.
    static TestTrajectory hexagon_start_stop(int laps, double duration);
};

inline std::ostream& operator<<(std::ostream& os, const TestTrajectory& param)
{
    return os << param.description;
}

} // namespace invariant::test

#endif // INVARIANT_TEST_TRAJECTORIES_H
