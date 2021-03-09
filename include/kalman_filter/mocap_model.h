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
    MocapModel(const ugl::lie::Pose& offset, const ugl::Matrix<6,6>& noise_covariance);

    /// @brief The observation function h(X,w)
    /// @param state the system state
    /// @param noise the measurement noise (default: zero vector)
    /// @return The expected observation given the system state
    ugl::lie::Pose h(const ugl::lie::ExtendedPose& state, const ugl::lie::Pose::VectorType& noise=ugl::lie::Pose::VectorType::Zero()) const;

    /// @brief Left group action on a target representing measurement y = actor * target
    /// @param actor the value which is to act on target
    /// @param target the target of the action
    /// @return Result of action: y = actor * target
    static ugl::lie::Pose group_action(const ugl::lie::ExtendedPose& actor, const ugl::lie::Pose& target);

    /// @brief Target for group action
    /// @return Const reference to target variable
    const ugl::lie::Pose& target() const;

    /// @brief Error jacobian of measurement model
    /// @return Const reference to the jacobian matrix
    ugl::Matrix<6,9> error_jacobian() const;

    /// @brief Noise jacobian of measurement model
    /// @return Const reference to the jacobian matrix
    ugl::Matrix<6,6> noise_jacobian() const;

    /// @brief Noise covariance of measurement model
    /// @return Const reference to the covariance matrix
    const ugl::Matrix<6,6>& noise_covariance() const;

    /// @brief N_hat = E*N*E^T
    const ugl::Matrix<6,6>& modified_noise_covariance() const;

private:
    ugl::lie::Pose m_target;
    ugl::Matrix<6,6> m_noise_covariance;
    ugl::Matrix<6,6> m_modified_noise_covariance;
};

} // namespace invariant

#endif // INVARIANT_MOCAP_MODEL_H
