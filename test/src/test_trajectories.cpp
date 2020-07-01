#include "test_trajectories.h"

#include <ugl/math/vector.h>
#include <ugl/math/quaternion.h>
#include <ugl/trajectory/trajectory.h>
#include <ugl/trajectory/bezier.h>
#include <ugl/trajectory/slerp_segment.h>

namespace invariant::test
{

ugl::trajectory::Trajectory getStandStillTrajectory(double duration)
{
    ugl::trajectory::Bezier<0> lin_traj{duration, {ugl::Vector3::Zero()}};

    ugl::trajectory::SlerpSegment ang_traj{duration, ugl::UnitQuaternion::Identity(), ugl::UnitQuaternion::Identity()};

    return {lin_traj, ang_traj};
}

} // namespace invariant::test
