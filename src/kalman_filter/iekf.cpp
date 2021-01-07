#include "iekf.h"

#include <ugl/math/vector.h>
#include <ugl/math/matrix.h>

#include <ugl/lie_group/euclidean.h>
#include <ugl/lie_group/rotation.h>
#include <ugl/lie_group/pose.h>
#include <ugl/lie_group/extended_pose.h>

#include "mocap_model.h"
#include "gps_model.h"

namespace invariant
{

using namespace ugl;

const IEKF::Covariance<9> IEKF::s_default_covariance = []() {
    constexpr double rot_stddev = 1.0;  // [rad]
    constexpr double vel_stddev = 1.5;  // [m/s]
    constexpr double pos_stddev = 2.0;  // [m]

    Covariance<9> covariance = Covariance<9>::Zero();
    covariance.block<3,3>(0,0) = Matrix3::Identity() * rot_stddev*rot_stddev;
    covariance.block<3,3>(3,3) = Matrix3::Identity() * vel_stddev*vel_stddev;
    covariance.block<3,3>(6,6) = Matrix3::Identity() * pos_stddev*pos_stddev;

    return covariance;
}();

IEKF::IEKF(const lie::Rotation& R0, const Vector3& p0, const Vector3& v0, const Covariance<9>& P0)
    : m_X(R0, v0, p0)
    , m_P(P0)
{
}

void IEKF::predict(double dt, const Vector3& acc, const Vector3& ang_vel)
{
    const lie::Rotation R = m_X.rotation();
    const Vector3 v = m_X.velocity();
    const Vector3 p = m_X.position();

    const lie::Rotation R_pred = R * lie::SO3::exp(ang_vel*dt);
    const Vector3 v_pred = v + (R*acc + s_gravity)*dt;
    const Vector3 p_pred = p + v*dt + 0.5*(R*acc + s_gravity)*dt*dt;

    m_X = lie::ExtendedPose{R_pred, v_pred, p_pred};

    const auto& A = process_error_jacobian(acc, ang_vel);
    static const auto& D = process_noise_jacobian();
    static const auto& Q = process_noise_covariance();
    static const Matrix<9,9> Q_hat = D*Q*D.transpose();

    const Matrix<9,9> Phi = Matrix<9,9>::Identity() + A*dt; // Approximates Phi = exp(A*dt)
    m_P = Phi * (m_P + Q_hat*dt*dt) * Phi.transpose();
}

void IEKF::mocap_update(const lie::Pose& y)
{
    static const auto& H = MocapModel::error_jacobian();
    static const auto& E = MocapModel::noise_jacobian();
    static const auto& N = MocapModel::noise_covariance();
    static const Matrix<6,6> N_hat = E*N*E.transpose();

    const Matrix<6,6> S = H * m_P * H.transpose() + N_hat;
    const Matrix<9,6> K = m_P * H.transpose() * S.inverse();

    const lie::Pose innovation = MocapModel::group_action(m_X.inverse(), y) * MocapModel::target().inverse();
    const Vector<9> correction = K*lie::log(innovation);

    m_X = m_X * lie::exp(correction);
    m_P = (Covariance<9>::Identity() - K*H) * m_P;
}

void IEKF::gps_update(const lie::Euclidean<3>& y)
{
    static const auto& H = GpsModel::error_jacobian();
    static const auto& E = GpsModel::noise_jacobian();
    static const auto& N = GpsModel::noise_covariance();
    static const Matrix<3,3> N_hat = E*N*E.transpose();

    const Matrix<3,3> S = H * m_P * H.transpose() + N_hat;
    const Matrix<9,3> K = m_P * H.transpose() * S.inverse();

    const lie::Euclidean<3> innovation = GpsModel::group_action(m_X.inverse(), y) * GpsModel::target().inverse();
    const Vector<9> correction = K*lie::log(innovation);

    m_X = m_X * lie::exp(correction);
    m_P = (Covariance<9>::Identity() - K*H) * m_P;
}

IEKF::Jacobian<9,9> IEKF::process_error_jacobian(const Vector3& acc, const Vector3& ang_vel)
{
    Jacobian<9,9> jac = Jacobian<9,9>::Zero();
    jac.block<3,3>(kRotIndex,kRotIndex) = -lie::skew(ang_vel);
    jac.block<3,3>(kVelIndex,kVelIndex) = -lie::skew(ang_vel);
    jac.block<3,3>(kPosIndex,kPosIndex) = -lie::skew(ang_vel);
    jac.block<3,3>(kVelIndex,kRotIndex) = -lie::skew(acc);
    jac.block<3,3>(kPosIndex,kVelIndex) = Matrix3::Identity();
    return jac;
}

IEKF::Jacobian<9,6> IEKF::process_noise_jacobian()
{
    Jacobian<9,6> jac = Jacobian<9,6>::Zero();
    jac.block<3,3>(0,0) = Matrix3::Identity();
    jac.block<3,3>(3,3) = Matrix3::Identity();
    return jac;
}

IEKF::Covariance<6> IEKF::process_noise_covariance()
{
    constexpr double sigma_gyro  = 0.1 ; // [rad/s]
    constexpr double sigma_accel = 5.0;  // [m/s^2]
    Covariance<6> covar = Covariance<6>::Zero();
    covar.block<3,3>(0,0) = Matrix3::Identity() * sigma_gyro*sigma_gyro;
    covar.block<3,3>(3,3) = Matrix3::Identity() * sigma_accel*sigma_accel;
    return covar;
}

const Vector3 IEKF::s_gravity{0.0, 0.0, -9.82};

} // namespace invariant
