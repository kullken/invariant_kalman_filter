#ifndef INVARIANT_MOCAP_MODEL_H
#define INVARIANT_MOCAP_MODEL_H

#include <ugl/math/matrix.h>
#include <ugl/lie_group/pose.h>
#include <ugl/lie_group/extended_pose.h>

namespace invariant
{

/// @brief A measurement model for motion-capture sensors.
class MocapModel
{
public:
    MocapModel() = delete;

    /// @brief Left group action on a target representing measurement y = actor * target
    /// @param actor the value which is to act on target
    /// @param target the target of the action
    /// @return Result of action: y = actor * target
    static ugl::lie::Pose group_action(const ugl::lie::ExtendedPose& actor, const ugl::lie::Pose& target);

    /// @brief Target for group action
    /// @return Const reference to target variable
    static const ugl::lie::Pose& target();

    /// @brief Error jacobian of measurement model
    /// @return Const reference to the jacobian matrix
    static const ugl::Matrix<6,9>& H();

    /// @brief Noise jacobian of measurement model
    /// @return Const reference to the jacobian matrix
    static const ugl::Matrix<6,6>& E();

    /// @brief Noise covariance of measurement model
    /// @return Const reference to the covariance matrix
    static const ugl::Matrix<6,6>& N();

private:
    static const ugl::lie::Pose s_target;
    static const ugl::Matrix<6,9> s_H;
    static const ugl::Matrix<6,6> s_E;
    static const ugl::Matrix<6,6> s_N;
};

} // namespace invariant

#endif // INVARIANT_MOCAP_MODEL_H
