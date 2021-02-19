#ifndef INVARIANT_IEKF_H
#define INVARIANT_IEKF_H

#include <ugl/math/vector.h>
#include <ugl/math/matrix.h>
#include <ugl/math/quaternion.h>

#include <ugl/lie_group/euclidean.h>
#include <ugl/lie_group/rotation.h>
#include <ugl/lie_group/pose.h>
#include <ugl/lie_group/extended_pose.h>

#include "gps_model.h"
#include "mocap_model.h"

namespace invariant
{

class IEKF
{
public:
    template<int n>
    using Covariance = ugl::Matrix<n, n>;

    template<int rows, int cols>
    using Jacobian = ugl::Matrix<rows, cols>;

public:
    IEKF() = default;
    IEKF(const ugl::lie::ExtendedPose& X0, const Covariance<9>& P0);
    IEKF(const ugl::lie::Rotation& R0, const ugl::Vector3& p0, const ugl::Vector3& v0, const Covariance<9>& P0);

    ugl::Vector3 get_pos() const { return m_X.position(); }
    ugl::Vector3 get_vel() const { return m_X.velocity(); }
    ugl::lie::Rotation get_rot() const { return m_X.rotation(); }
    ugl::UnitQuaternion get_quat() const { return m_X.rotation().to_quaternion(); }
    const ugl::lie::ExtendedPose& get_state() const { return m_X; }
    const Covariance<9>& get_covariance() const { return m_P; }

    void set_pos(const ugl::Vector3& pos) { m_X.set_position(pos); }
    void set_vel(const ugl::Vector3& vel) { m_X.set_velocity(vel); }
    void set_rot(const ugl::lie::Rotation& rot) { m_X.set_rotation(rot); }
    void set_quat(const ugl::UnitQuaternion& quat) { m_X.set_rotation(ugl::lie::Rotation{quat}); }
    void set_state(const ugl::lie::ExtendedPose& state) { m_X = state; }
    void set_covariance(const Covariance<9>& P) { m_P = P; }

    void predict(double dt, const ugl::Vector3& acc, const ugl::Vector3& ang_vel);
    void update(const ugl::lie::Pose& y, const MocapModel& sensor_model);
    void update(const ugl::lie::Euclidean<3>& y, const GpsModel& sensor_model);

    template<typename MeasurementModel>
    void update(const typename MeasurementModel::MeasurementType& y);

private:
    /// @brief Error jacobian of the process model
    static Jacobian<9,9> process_error_jacobian(const ugl::Vector3& acc, const ugl::Vector3& ang_vel);

    /// @brief Noise jacobian of the process model
    static Jacobian<9,6> process_noise_jacobian();

    /// @brief Noise covariance of the process model
    static Covariance<6> process_noise_covariance();

private:
    ugl::lie::ExtendedPose m_X = ugl::lie::ExtendedPose::Identity();
    Covariance<9> m_P = s_default_covariance;

    static const ugl::Matrix<9,9> s_default_covariance;

    static constexpr int kRotIndex = 0;
    static constexpr int kVelIndex = 3;
    static constexpr int kPosIndex = 6;

    // Gravitational acceleration
    static const ugl::Vector3 s_gravity;
};

template<typename MeasurementModel>
void IEKF::update(const typename MeasurementModel::MeasurementType& y)
{
    using Y = typename MeasurementModel::MeasurementType;
    using G = ugl::lie::ExtendedPose;

    const auto& H = MeasurementModel::error_jacobian();
    const auto& E = MeasurementModel::noise_jacobian();
    const auto& N = MeasurementModel::noise_covariance();

    const ugl::Matrix<Y::DoF,Y::DoF> S = H * m_P * H.transpose() + E * N * E.transpose();
    const ugl::Matrix<G::DoF,Y::DoF> K = m_P * H.transpose() * S.inverse();

    const Y innovation = MeasurementModel::group_action(m_X.inverse(), y) * MeasurementModel::target().inverse();
    const G::VectorType correction = K*ugl::lie::log(innovation);

    m_X = m_X * ugl::lie::exp(correction);
    m_P = m_P - K*H*m_P;
}

} // namespace invariant

#endif // INVARIANT_IEKF_H
