#include "mekf.h"

#include <ugl/math/vector.h>
#include <ugl/math/matrix.h>
#include <ugl/lie_group/rotation.h>

#include "mekf_types.h"

namespace mekf
{

using ugl::Vector3;
using ugl::Matrix3;
using ugl::lie::Rotation;

const Vector3 MEKF::s_gravity{0.0, 0.0, -9.82};

MEKF::MEKF(const Rotation& R0, const Vector3& p0, const Vector3& v0, const Covariance<9>& P0)
    : m_x()
    , m_R_ref(R0)
    , m_P(P0)
{
    m_x << p0, v0, Vector3::Zero();
}

void MEKF::predict(double dt, const Vector3& acc, const Vector3& ang_vel)
{
    const Jacobian A = MEKF::state_transition_jac(m_R_ref, dt, acc, ang_vel);
    const Covariance<9> Q = MEKF::state_transition_var(dt);

    m_x = MEKF::state_transition_model(m_x, m_R_ref, dt, acc, ang_vel);
    m_P = A*m_P*A.transpose() + Q;

    reset_attitude_error();
}

void MEKF::mocap_update(const Rotation&, const Vector3&)
{

}

void MEKF::position_update(const Position& measurement)
{
    const PositionJacobian H = MEKF::position_measurement_jac();
    const Covariance<3> R = MEKF::position_measurement_var();

    const Position y = measurement - MEKF::position_measurement_model(m_x);
    const Covariance<3> S = H*m_P*H.transpose() + R;
    const ugl::Matrix<9, 3> K = m_P*H.transpose()*S.inverse();

    m_x = m_x + K*y;
    m_P = (Covariance<9>::Identity() - K*H) * m_P;

    reset_attitude_error();
}

void MEKF::reset_attitude_error()
{
    const Vector3 delta = m_x.segment<3>(6);

    ugl::Matrix<9, 9> T = ugl::Matrix<9, 9>::Identity();
    T.bottomRightCorner<3,3>() = (-1/2 * ugl::lie::SO3::exp(delta).matrix());

    m_P = T * m_P * T.transpose();
    m_R_ref *= ugl::lie::SO3::exp(delta);
    m_x.segment<3>(6) = Vector3::Zero();
}

// State transition model without resetting attitude error.
State MEKF::state_transition_model(const State& x, const Rotation& R_ref, double dt, const Vector3& acc, const Vector3& ang_vel)
{
    Vector3 pos = x.segment<3>(0);
    Vector3 vel = x.segment<3>(3);
    Vector3 delta = x.segment<3>(6);

    const Rotation R_actual = R_ref * ugl::lie::SO3::exp(delta);
    // const Vector3 rotated_acc = R_actual * acc;                  // Without inversion of R
    const Vector3 rotated_acc = R_actual.inverse() * acc;        // With inversion of R

    pos   += dt * vel + dt*dt/2 * (rotated_acc + s_gravity);
    vel   += dt * (rotated_acc + s_gravity);
    delta += dt * (ang_vel - 1/2 * ugl::lie::skew(ang_vel)*delta);

    State x_predicted;
    x_predicted << pos, vel, delta;

    return x_predicted;
}

Jacobian MEKF::state_transition_jac(const Rotation& R_ref, double dt, const Vector3& acc, const Vector3& ang_vel)
{
    Jacobian jac = Jacobian::Identity();
    jac.block<3,3>(0,3) = dt * Matrix3::Identity();
    // jac.block<3,3>(3,6) = dt * ugl::lie::skew(R_ref*acc);                  // Without inversion of R
    jac.block<3,3>(3,6) = dt * ugl::lie::skew(R_ref.inverse()*acc);        // With inversion of R
    jac.block<3,3>(6,6) -= dt * 1/2 * ugl::lie::skew(ang_vel);

    return jac;
}

Covariance<9> MEKF::state_transition_var(double dt)
{
    // Values directly from Mueller et al. (2018).
    constexpr double sigma_acc  = 5;      // m/s^2
    constexpr double sigma_rate = 0.1;    // rad/s

    constexpr double sigma_acc_sq  = sigma_acc*sigma_acc;
    constexpr double sigma_rate_sq = sigma_rate*sigma_rate;

    const double dt2 = dt*dt;
    const double dt3 = dt2*dt;
    const double dt4 = dt3*dt;

    Covariance<9> Q = Covariance<9>::Zero();
    Q.block<3,3>(0,0) = Matrix3::Identity() * dt4/4 * sigma_acc_sq;
    Q.block<3,3>(0,3) = Matrix3::Identity() * dt3/2 * sigma_acc_sq;
    Q.block<3,3>(3,0) = Matrix3::Identity() * dt3/2 * sigma_acc_sq;
    Q.block<3,3>(3,3) = Matrix3::Identity() * dt2 * sigma_acc_sq;
    Q.block<3,3>(6,6) = Matrix3::Identity() * dt2 * sigma_rate_sq;

    return Q;
}

Position MEKF::position_measurement_model(const State& x)
{
    return x.segment<3>(0);
}

PositionJacobian MEKF::position_measurement_jac()
{
    return PositionJacobian::Identity();
}

Covariance<3> MEKF::position_measurement_var()
{
    constexpr double sigma_pos = 0.05 * 100;       // m
    return Covariance<3>::Identity() * sigma_pos*sigma_pos;
}

}