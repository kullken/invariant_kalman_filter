#ifndef INVARIANT_GPS_MODEL_H
#define INVARIANT_GPS_MODEL_H

#include <ugl/math/matrix.h>
#include <ugl/lie_group/extended_pose.h>

namespace invariant
{

/// @brief A measurement model for GPS sensors.
class GpsModel
{
public:
    GpsModel() = delete;

    /// @brief The observation function h(X)
    /// @param state the system state
    /// @return The expected observation given the system state
    static ugl::Vector3 h(const ugl::lie::ExtendedPose& state);

    /// @brief Left group action on a target representing measurement y = actor * target
    /// @param actor the value which is to act on target
    /// @param target the target of the action
    /// @return Result of action: y = actor * target
    static ugl::Vector3 group_action(const ugl::lie::ExtendedPose& actor, const ugl::Vector3& target);

    /// @brief Target for group action
    /// @return Const reference to target variable
    static const ugl::Vector3& target();

    /// @brief Error jacobian of measurement model
    /// @return Const reference to the jacobian matrix
    static const ugl::Matrix<3,9>& error_jacobian();

    /// @brief Noise jacobian of measurement model
    /// @return Const reference to the jacobian matrix
    static const ugl::Matrix<3,3>& noise_jacobian();

    /// @brief Noise covariance of measurement model
    /// @return Const reference to the covariance matrix
    static const ugl::Matrix<3,3>& noise_covariance();

private:
    static const ugl::Vector3 s_target;
    static const ugl::Matrix<3,9> s_error_jacobian;
    static const ugl::Matrix<3,3> s_noise_jacobian;
    static const ugl::Matrix<3,3> s_noise_covariance;
};

} // namespace invariant

#endif // INVARIANT_GPS_MODEL_H
