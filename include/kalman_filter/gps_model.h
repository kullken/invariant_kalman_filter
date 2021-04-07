#ifndef INVARIANT_GPS_MODEL_H
#define INVARIANT_GPS_MODEL_H

#include <ugl/math/matrix.h>
#include <ugl/lie_group/euclidean.h>
#include <ugl/lie_group/extended_pose.h>

namespace invariant
{

/// @brief A measurement model for GPS sensors.
class GpsModel
{
public:
    using MeasurementType = ugl::lie::Euclidean<3>;

public:
    GpsModel() : GpsModel(MeasurementType::Identity(), ugl::Matrix<3,3>::Identity()) {}

    GpsModel(const MeasurementType& offset, const ugl::Matrix<3,3>& noise_covariance);

    /// @brief The observation function h(X,w)
    /// @param state the system state
    /// @param noise the measurement noise (default: zero vector)
    /// @return The expected observation given the system state
    ugl::Vector3 h(const ugl::lie::ExtendedPose& state, const ugl::Vector3& noise=ugl::Vector3::Zero()) const;

    /// @brief Left group action on a target representing measurement y = actor * target
    /// @param actor the value which is to act on target
    /// @param target the target of the action
    /// @return Result of action: y = actor * target
    static MeasurementType group_action(const ugl::lie::ExtendedPose& actor, const MeasurementType& target);

    /// @brief Target for group action
    /// @return Const reference to target variable
    const MeasurementType& target() const;

    /// @brief Error jacobian of measurement model
    /// @return Const reference to the jacobian matrix
    const ugl::Matrix<3,9>& error_jacobian() const;

    /// @brief Noise jacobian of measurement model
    /// @return Const reference to the jacobian matrix
    static const ugl::Matrix<3,3>& noise_jacobian();

    /// @brief Noise covariance of measurement model
    /// @return Const reference to the covariance matrix
    const ugl::Matrix<3,3>& noise_covariance() const;

    /// @brief N_hat = E*N*E^T
    const ugl::Matrix<3,3>& modified_noise_covariance() const;

private:
    MeasurementType m_target;
    ugl::Matrix<3,3> m_noise_covariance;
    ugl::Matrix<3,3> m_modified_noise_covariance;
    ugl::Matrix<3,9> m_error_jacobian;

    static const ugl::Matrix<3,3> s_noise_jacobian;
};

} // namespace invariant

#endif // INVARIANT_GPS_MODEL_H
