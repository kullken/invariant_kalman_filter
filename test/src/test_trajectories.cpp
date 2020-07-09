#include "test_trajectories.h"

#include <algorithm>
#include <vector>

#include <ugl/math/vector.h>
#include <ugl/math/matrix.h>
#include <ugl/math/quaternion.h>

#include <ugl/trajectory/trajectory.h>
#include <ugl/trajectory/bezier.h>
#include <ugl/trajectory/slerp_segment.h>
#include <ugl/trajectory/slerp_sequence.h>

namespace invariant::test
{
namespace 
{

constexpr auto pi = 3.141592653589793238462643383279502884L;
constexpr auto deg2rad = pi / 180.0;

static auto rotate_yaw(double degrees, double total_duration)
{
    const ugl::Vector3 rotation_axis = degrees < 0 ? ugl::Vector3{0,0,-1} : ugl::Vector3{0,0,1};
    double degrees_left = degrees < 0 ? -degrees : degrees;
    const double deg_per_sec = degrees_left / total_duration;

    std::vector<ugl::trajectory::SlerpSegment> segments{};
    ugl::UnitQuaternion start = ugl::UnitQuaternion::Identity();
    ugl::UnitQuaternion end   = ugl::UnitQuaternion::Identity();

    while (degrees_left > 0)
    {
        const double delta_deg = std::min(degrees_left, 90.0);
        const ugl::UnitQuaternion delta = ugl::math::to_quat(delta_deg*deg2rad, rotation_axis);
        end = delta * start;

        const double duration = delta_deg / deg_per_sec;
        segments.emplace_back(duration, start, end);

        start = end;
        degrees_left -= delta_deg;
    }
    
    return ugl::trajectory::SlerpSequence(segments);
}

} // namespace

ugl::trajectory::Trajectory getStandStillTrajectory(double duration)
{
    ugl::trajectory::Bezier<0> lin_traj{duration, {ugl::Vector3::Zero()}};

    ugl::trajectory::SlerpSegment ang_traj{duration, ugl::UnitQuaternion::Identity(), ugl::UnitQuaternion::Identity()};

    return {lin_traj, ang_traj};
}

ugl::trajectory::Trajectory rotate_in_place(double degrees, double duration)
{
    ugl::trajectory::Bezier<0> lin_traj{duration, {ugl::Vector3::Zero()}};

    auto ang_traj = rotate_yaw(degrees, duration);

    return {lin_traj, ang_traj};
}

ugl::trajectory::Trajectory straight_line(ugl::Vector3 delta, double duration)
{
    ugl::trajectory::Bezier<1> lin_traj{duration, {ugl::Vector3::Zero(), delta}};

    ugl::trajectory::SlerpSegment ang_traj{duration, ugl::UnitQuaternion::Identity(), ugl::UnitQuaternion::Identity()};

    return {lin_traj, ang_traj};
}

ugl::trajectory::Trajectory quadratic_translation(ugl::Vector3 delta, double duration)
{
    ugl::trajectory::Bezier<2> lin_traj{duration, {ugl::Vector3::Zero(), ugl::Vector3::Zero(), delta}};

    ugl::trajectory::SlerpSegment ang_traj{duration, ugl::UnitQuaternion::Identity(), ugl::UnitQuaternion::Identity()};

    return {lin_traj, ang_traj};
}

} // namespace invariant::test
