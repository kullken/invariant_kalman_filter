#include "test_trajectories.h"

#include <algorithm>
#include <sstream>
#include <vector>

#include <ugl/math/vector.h>
#include <ugl/math/matrix.h>
#include <ugl/math/quaternion.h>

#include <ugl/trajectory/trajectory.h>
#include <ugl/trajectory/circle_arc.h>
#include <ugl/trajectory/helix.h>
#include <ugl/trajectory/bezier.h>
#include <ugl/trajectory/bezier_sequence.h>
#include <ugl/trajectory/slerp_segment.h>
#include <ugl/trajectory/slerp_sequence.h>

namespace invariant::test
{
namespace
{

constexpr double deg2rad(double degrees)
{
    constexpr auto pi = 3.141592653589793238462643383279502884L;
    return degrees * pi / 180.0;
}

auto rotate_yaw(double degrees, double total_duration, double start_degree=0.0)
{
    const ugl::Vector3 rotation_axis = degrees < 0 ? ugl::Vector3{0,0,-1} : ugl::Vector3{0,0,1};
    double degrees_left = degrees < 0 ? -degrees : degrees;
    const double deg_per_sec = degrees_left / total_duration;

    std::vector<ugl::trajectory::SlerpSegment> segments{};
    ugl::UnitQuaternion start = ugl::math::to_quat(deg2rad(start_degree), ugl::Vector3{0,0,1});
    ugl::UnitQuaternion end;

    while (degrees_left > 0)
    {
        const double delta_deg = std::min(degrees_left, 90.0);
        const ugl::UnitQuaternion delta = ugl::math::to_quat(deg2rad(delta_deg), rotation_axis);
        end = delta * start;

        const double duration = delta_deg / deg_per_sec;
        segments.emplace_back(duration, start, end);

        start = end;
        degrees_left -= delta_deg;
    }

    return ugl::trajectory::SlerpSequence(segments);
}

} // namespace

TestTrajectory TestTrajectory::stand_still(double duration)
{
    std::stringstream ss;
    ss << "StandStill: " << duration << "s";

    ugl::trajectory::Bezier<0> lin_traj{duration, {ugl::Vector3::Zero()}};

    ugl::trajectory::SlerpSegment ang_traj{duration, ugl::UnitQuaternion::Identity(), ugl::UnitQuaternion::Identity()};

    return TestTrajectory{ss.str(), ugl::trajectory::Trajectory{lin_traj, ang_traj}};
}

TestTrajectory TestTrajectory::rotate_in_place(double degrees, double duration)
{
    std::stringstream ss;
    ss << "Rotate: " << degrees << ", " << duration << "s";

    ugl::trajectory::Bezier<0> lin_traj{duration, {ugl::Vector3::Zero()}};

    auto ang_traj = rotate_yaw(degrees, duration);

    return TestTrajectory{ss.str(), ugl::trajectory::Trajectory{lin_traj, ang_traj}};
}

TestTrajectory TestTrajectory::constant_velocity(ugl::Vector3 velocity, double duration)
{
    std::stringstream ss;
    ss << "ConstantVel: " << "[" << velocity.x() << "," << velocity.y() << "," << velocity.z() << "] m/s, " << duration << "s";

    ugl::trajectory::Bezier<1> lin_traj{duration, {ugl::Vector3::Zero(), velocity*duration}};

    ugl::trajectory::SlerpSegment ang_traj{duration, ugl::UnitQuaternion::Identity(), ugl::UnitQuaternion::Identity()};

    return TestTrajectory{ss.str(), ugl::trajectory::Trajectory{lin_traj, ang_traj}};
}

TestTrajectory TestTrajectory::start_stop(ugl::Vector3 acceleration, double duration)
{
    std::stringstream ss;
    ss << "StartStop: " << "[" << acceleration.x() << "," << acceleration.y() << "," << acceleration.z() << "] m/s^2, " << duration << "s";

    const ugl::Vector3 start = ugl::Vector3::Zero();
    const ugl::Vector3 mid   = std::pow((duration/2), 2) * acceleration / 2;
    const ugl::Vector3 stop  = 2 * mid;

    const auto part1    = ugl::trajectory::Bezier<2>{duration/2, {start, start, mid}};
    const auto part2    = ugl::trajectory::Bezier<2>{duration/2, {mid, stop, stop}};
    const auto lin_traj = ugl::trajectory::BezierSequence<2>{{part1, part2}};

    ugl::trajectory::SlerpSegment ang_traj{duration, ugl::UnitQuaternion::Identity(), ugl::UnitQuaternion::Identity()};

    return TestTrajectory{ss.str(), ugl::trajectory::Trajectory{lin_traj, ang_traj}};
}

TestTrajectory TestTrajectory::circle(double degrees, double radius, double duration)
{
    std::stringstream ss;
    ss << "Circle: " << degrees << ", " << radius << "m, " << duration << "s";

    auto lin_traj = ugl::trajectory::CircleArc{deg2rad(degrees), radius, duration};
    auto ang_traj = rotate_yaw(degrees, duration, 90.0);

    return TestTrajectory{ss.str(), ugl::trajectory::Trajectory{lin_traj, ang_traj}};
}

TestTrajectory TestTrajectory::helix(double degrees, double radius, double z_velocity, double duration)
{
    std::stringstream ss;
    ss << "Helix: " << degrees << ", " << radius << "m, " << z_velocity << "m/s, " << duration << "s";

    auto lin_traj = ugl::trajectory::Helix{deg2rad(degrees), radius, duration, z_velocity};
    auto ang_traj = rotate_yaw(degrees, duration, 90.0);

    return TestTrajectory{ss.str(), ugl::trajectory::Trajectory{lin_traj, ang_traj}};
}

} // namespace invariant::test
