#include "accuracy_test.h"

#include <ugl/trajectory/trajectory.h>

namespace invariant::test
{

AccuracyResult compute_accuracy(const IEKF &filter, const ugl::trajectory::Trajectory &traj)
{
    AccuracyResult result;

    // TODO: How to deal with sensors at different Hz?
    // for (double t : range(0, trajectoty_.duration())) // TODO: Pseudo-code
    {
        // TODO: Update filter with IMU data.
        // TODO: Update filter with Mocap data.
        // TODO: Update result variable.
    }
    return result;
}

} // namespace invariant::test
