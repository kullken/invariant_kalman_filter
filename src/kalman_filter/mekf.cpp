#include "mekf.h"

#include <ugl/math/vector.h>
#include <ugl/math/matrix.h>
#include <ugl/lie_group/rotation.h>
#include <ugl/lie_group/pose.h>
#include <ugl/lie_group/extended_pose.h>

#include "gps_model.h"

namespace invariant
{

using ugl::Vector3;
using ugl::Matrix3;
using ugl::lie::Rotation;

const Vector3 MEKF::s_gravity{0.0, 0.0, -9.82};

const MEKF::Covariance<9> MEKF::s_default_covariance = []() {
    constexpr double rot_stddev = 0.1;  // [rad]
    constexpr double vel_stddev = 0.5;  // [m/s]
    constexpr double pos_stddev = 0.5;  // [m]

    Covariance<9> covariance = Covariance<9>::Zero();
    covariance.block<3,3>(0,0) = Matrix3::Identity() * rot_stddev*rot_stddev;
    covariance.block<3,3>(3,3) = Matrix3::Identity() * vel_stddev*vel_stddev;
    covariance.block<3,3>(6,6) = Matrix3::Identity() * pos_stddev*pos_stddev;

    return covariance;
}();

MEKF::MEKF(const Rotation& R0, const Vector3& p0, const Vector3& v0, const Covariance<9>& P0)
    : m_R_ref(R0)
    , m_P(P0)
{
    set_pos(p0);
    set_vel(v0);
}

ugl::lie::ExtendedPose MEKF::get_state() const
{
    return ugl::lie::ExtendedPose{get_rot(), get_vel(), get_pos()};
}

void MEKF::set_state(const ugl::lie::ExtendedPose& state)
{
    m_x.segment<3>(kPosIndex) = state.position();
    m_x.segment<3>(kVelIndex) = state.velocity();
    m_x.segment<3>(kRotIndex) = ugl::Vector3::Zero();
    m_R_ref = state.rotation();
}

void MEKF::predict(double dt, const Vector3& acc, const Vector3& ang_vel)
{
    m_x = MEKF::state_transition_model(m_x, m_R_ref, dt, acc, ang_vel);

    const auto& A = process_error_jacobian(m_R_ref, acc, ang_vel);
    const auto& D = process_noise_jacobian();
    const auto& Q = process_noise_covariance();

    const ugl::Matrix<9,9> Adt = A*dt;
    const ugl::Matrix<9,9> Adt2 = Adt*Adt;
    const ugl::Matrix<9,9> Phi = ugl::Matrix<9,9>::Identity() + Adt + Adt2/2 + Adt2*Adt/6 + Adt2*Adt2/24; // Approximates Phi = exp(A*dt)
    m_P = Phi*m_P*Phi.transpose() + Phi*D*Q*D.transpose()*Phi.transpose() * dt*dt;

    reset_attitude_error();
}

void MEKF::mocap_update(const ugl::lie::Pose&)
{

}

void MEKF::gps_update(const Vector3& y)
{
    const auto& H = GpsModel::error_jacobian();
    const auto& E = GpsModel::noise_jacobian();
    const auto& N = GpsModel::noise_covariance();

    const Covariance<3> S = H * m_P * H.transpose() + E * N * E.transpose();
    const ugl::Matrix<9,3> K = m_P * H.transpose() * S.inverse();

    const Vector3 innovation = y - GpsModel::h(get_state());

    m_x = m_x + K*innovation;
    m_P = (Covariance<9>::Identity() - K*H) * m_P;

    reset_attitude_error();
}

void MEKF::reset_attitude_error()
{
    const Vector3 delta = m_x.segment<3>(kRotIndex);

    ugl::Matrix<9,9> T = ugl::Matrix<9,9>::Zero();
    T.block<3,3>(kPosIndex,kPosIndex) = Matrix3::Identity();
    T.block<3,3>(kVelIndex,kVelIndex) = Matrix3::Identity();
    T.block<3,3>(kRotIndex,kRotIndex) = -0.5 * ugl::lie::SO3::exp(delta).matrix();

    m_P = T * m_P * T.transpose();
    m_R_ref *= ugl::lie::SO3::exp(delta);
    m_x.segment<3>(kRotIndex) = Vector3::Zero();
}

// State transition model without resetting attitude error.
MEKF::State MEKF::state_transition_model(const State& x, const Rotation& R_ref, double dt, const Vector3& acc, const Vector3& ang_vel)
{
    const Vector3 delta = x.segment<3>(kRotIndex);
    const Vector3 vel = x.segment<3>(kVelIndex);
    const Vector3 pos = x.segment<3>(kPosIndex);

    // const Rotation R = R_ref * ugl::lie::SO3::exp(delta);               // Without inversion of R
    const Rotation R = (R_ref * ugl::lie::SO3::exp(delta)).inverse();   // With inversion of R

    const Vector3 delta_pred = delta + (ang_vel - 0.5*ugl::lie::skew(ang_vel)*delta)*dt;
    const Vector3 vel_pred   = vel   + (R*acc + s_gravity)*dt;
    const Vector3 pos_pred   = pos   + vel*dt + (R*acc + s_gravity)*dt*dt*0.5;

    State x_pred;
    x_pred.segment<3>(kRotIndex) = delta_pred;
    x_pred.segment<3>(kVelIndex) = vel_pred;
    x_pred.segment<3>(kPosIndex) = pos_pred;

    return x_pred;
}

MEKF::Jacobian<9,9> MEKF::process_error_jacobian(const Rotation& R_ref, const Vector3& acc, const Vector3& ang_vel)
{
    Jacobian<9,9> jac = Jacobian<9,9>::Zero();
    jac.block<3,3>(kPosIndex,kVelIndex) = Matrix3::Identity();
    // jac.block<3,3>(kVelIndex,kRotIndex) = ugl::lie::skew(R_ref*acc);                  // Without inversion of R
    jac.block<3,3>(kVelIndex,kRotIndex) = ugl::lie::skew(R_ref.inverse()*acc);        // With inversion of R
    jac.block<3,3>(kRotIndex,kRotIndex) = -0.5 * ugl::lie::skew(ang_vel);
    return jac;
}

MEKF::Jacobian<9,6> MEKF::process_noise_jacobian()
{
    Jacobian<9,6> jac = Jacobian<9,6>::Zero();
    jac.block<3,3>(0,0) = Matrix3::Identity();
    jac.block<3,3>(3,3) = Matrix3::Identity();
    return jac;
}

MEKF::Covariance<6> MEKF::process_noise_covariance()
{
    constexpr double sigma_gyro  = 0.1 ; // [rad/s]
    constexpr double sigma_accel = 5.0;  // [m/s^2]
    Covariance<6> covar = Covariance<6>::Zero();
    covar.block<3,3>(0,0) = Matrix3::Identity() * sigma_gyro*sigma_gyro;
    covar.block<3,3>(3,3) = Matrix3::Identity() * sigma_accel*sigma_accel;
    return covar;
}

} // namespace invariant
