#include "mekf.h"

#include <unsupported/Eigen/MatrixFunctions>

#include <ugl/math/vector.h>
#include <ugl/math/matrix.h>

#include "mekf_types.h"

namespace mekf
{

using ugl::Vector3;
using ugl::Matrix3;
using ugl::Rotation;

/// Skew-symmetric cross-product matrix of a 3D vector.
Matrix3 S(const Vector3& vec)
{
    Matrix3 mat;
    mat << 0.0, -vec(2), vec(1), 
           vec(2), 0.0, -vec(0), 
           -vec(1), vec(0), 0.0;
    return mat;
}

const Vector3 MEKF::s_gravity{0.0, 0.0, -9.82};

MEKF::MEKF(const Vector3& initial_pos, const Vector3& initial_vel, const Rotation& initial_rot, const Covariance& initial_covar)
    : m_x()
    , m_R_ref(initial_rot)
    , m_P(initial_covar)
{
    m_x << initial_pos, initial_vel, Vector3::Zero();
}

void MEKF::predict(double dt, const Vector3& acc, const Vector3& ang_vel)
{
    const Jacobian A = MEKF::state_transition_jac(m_R_ref, dt, acc, ang_vel);
    const Covariance Q = MEKF::state_transition_var(dt);

    m_x = MEKF::state_transition_model(m_x, m_R_ref, dt, acc, ang_vel);
    m_P = A*m_P*A.transpose() + Q;

    reset_attitude_error();
}

void MEKF::update_with_position(const Position& measurement)
{
    const PositionJacobian H = MEKF::position_measurement_jac();
    const PositionCovariance R = MEKF::position_measurement_var();

    const Position y = measurement - MEKF::position_measurement_model(m_x);
    const PositionCovariance S = H*m_P*H.transpose() + R;
    const ugl::Matrix<9, 3> K = m_P*H.transpose()*S.inverse();

    m_x = m_x + K*y;
    m_P = (Covariance::Identity() - K*H) * m_P;

    reset_attitude_error();
}

void MEKF::reset_attitude_error()
{
    const Vector3 delta = m_x.segment<3>(6);

    ugl::Matrix<9, 9> T = ugl::Matrix<9, 9>::Identity();
    T.bottomRightCorner<3,3>() = (-1/2 * S(delta)).exp();

    m_P = T * m_P * T.transpose();
    m_R_ref *= (S(delta)).exp();
    m_x.segment<3>(6) = Vector3::Zero();
}

// State transition model without resetting attitude error.
State MEKF::state_transition_model(const State& x, const Rotation& R_ref, double dt, const Vector3& acc, const Vector3& ang_vel)
{
    Vector3 pos = x.segment<3>(0);
    Vector3 vel = x.segment<3>(3);
    Vector3 delta = x.segment<3>(6);

    // Vector3 rotated_acc = R_ref*S(delta).exp()*acc;                        // Without inversion of R
    Vector3 rotated_acc = (R_ref*S(delta).exp()).inverse()*acc;            // With inversion of R

    pos   += dt * vel + dt*dt/2 * (rotated_acc + s_gravity);
    vel   += dt * (rotated_acc + s_gravity);
    delta += dt * (ang_vel - 1/2 * S(ang_vel)*delta);

    State x_predicted;
    x_predicted << pos, vel, delta;

    return x_predicted;
}

Jacobian MEKF::state_transition_jac(const Rotation& R_ref, double dt, const Vector3& acc, const Vector3& ang_vel)
{
    Jacobian jac = Jacobian::Identity();
    jac.block<3,3>(0,3) = dt * Matrix3::Identity();
    // jac.block<3,3>(3,6) = dt * S(R_ref*acc);                  // Without inversion of R
    jac.block<3,3>(3,6) = dt * S(R_ref.inverse()*acc);        // With inversion of R
    jac.block<3,3>(6,6) -= dt * 1/2 * S(ang_vel);

    return jac;
}

Covariance MEKF::state_transition_var(double dt)
{
    // Values directly from Mueller et al. (2018).
    constexpr double sigma_acc  = 5;      // m/s^2
    constexpr double sigma_rate = 0.1;    // rad/s

    constexpr double sigma_acc_sq  = sigma_acc*sigma_acc;
    constexpr double sigma_rate_sq = sigma_rate*sigma_rate;

    const double dt2 = dt*dt;
    const double dt3 = dt2*dt;
    const double dt4 = dt3*dt;

    Covariance Q = Covariance::Zero();
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

PositionCovariance MEKF::position_measurement_var()
{
    constexpr double sigma_pos = 0.05 * 100;       // m
    return PositionCovariance::Identity() * sigma_pos*sigma_pos;
}

}