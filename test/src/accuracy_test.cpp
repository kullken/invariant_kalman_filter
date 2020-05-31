#include "accuracy_test.h"

#include <ugl/trajectory/trajectory.h>

namespace invariant::test
{

AccuracyTest::Result AccuracyTest::compute_accuracy(IEKF filter, const ugl::trajectory::Trajectory &traj)
{
    AccuracyTest::Result result;

    // TODO: How to deal with sensors at different Hz?
    // TODO: Check numerical precision of t. Perhaps use int+scale?
    const double dt = 0.01;
    for (double t = 0; t <= traj.duration(); t+=dt)
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
