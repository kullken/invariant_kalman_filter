#include "iekf.h"

#include <ugl/math/vector.h>
#include <ugl/math/matrix.h>

#include <ugl/lie_group/euclidean.h>
#include <ugl/lie_group/rotation.h>
#include <ugl/lie_group/pose.h>
#include <ugl/lie_group/extended_pose.h>

#include "gps_model.h"
#include "imu_model.h"
#include "mocap_model.h"

namespace invariant
{

using namespace ugl;

const Vector3 IEKF::s_gravity{0.0, 0.0, -9.82};

IEKF::IEKF(const lie::ExtendedPose& X0, const Covariance<9>& P0)
    : m_X(X0)
    , m_P(P0)
{
}

IEKF::IEKF(const lie::Rotation& R0, const Vector3& p0, const Vector3& v0, const Covariance<9>& P0)
    : m_X(R0, v0, p0)
    , m_P(P0)
{
}

void IEKF::predict(double dt, const Vector3& acc, const Vector3& ang_vel, const ImuModel& imu_model)
{
    const lie::Rotation R = m_X.rotation();
    const Vector3 v = m_X.velocity();
    const Vector3 p = m_X.position();

    const lie::Rotation R_pred = R * lie::SO3::exp(ang_vel*dt);
    const Vector3 v_pred = v + (R*acc + s_gravity)*dt;
    const Vector3 p_pred = p + v*dt + 0.5*(R*acc + s_gravity)*dt*dt;

    m_X = lie::ExtendedPose{R_pred, v_pred, p_pred};

    const auto& A = imu_model.error_jacobian(acc, ang_vel);
    const auto& Q_hat = imu_model.modified_noise_covariance();

    const Matrix<9,9> Phi = Matrix<9,9>::Identity() + A*dt; // Approximates Phi = exp(A*dt)
    m_P = Phi * (m_P + Q_hat*dt*dt) * Phi.transpose();
}

void IEKF::update(const lie::Pose& y, const MocapModel& sensor_model)
{
    const auto& H = sensor_model.error_jacobian();
    const auto& N_hat = sensor_model.modified_noise_covariance();

    const Matrix<6,6> S = H * m_P * H.transpose() + N_hat;
    const Matrix<9,6> K = m_P * H.transpose() * S.inverse();

    const auto innovation = lie::ominus(MocapModel::group_action(m_X.inverse(), y), sensor_model.target());

    m_X = lie::oplus(m_X, K*innovation);
    m_P = (Covariance<9>::Identity() - K*H) * m_P;
}

void IEKF::update(const lie::Euclidean<3>& y, const GpsModel& sensor_model)
{
    const auto& H = sensor_model.error_jacobian();
    const auto& N_hat = sensor_model.modified_noise_covariance();

    const Matrix<3,3> S = H * m_P * H.transpose() + N_hat;
    const Matrix<9,3> K = m_P * H.transpose() * S.inverse();

    const auto innovation = lie::ominus(GpsModel::group_action(m_X.inverse(), y), sensor_model.target());

    m_X = lie::oplus(m_X, K*innovation);
    m_P = (Covariance<9>::Identity() - K*H) * m_P;
}

} // namespace invariant
