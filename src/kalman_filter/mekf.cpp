#include "mekf.h"

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

const Vector3 MEKF::s_gravity{0.0, 0.0, -9.82};

MEKF::MEKF(const ugl::lie::ExtendedPose& X0, const Covariance<9>& P0)
    : m_P(P0)
{
    set_state(X0);
}

MEKF::MEKF(const lie::Rotation& R0, const Vector3& p0, const Vector3& v0, const Covariance<9>& P0)
    : m_R_ref(R0)
    , m_P(P0)
{
    set_pos(p0);
    set_vel(v0);
}

lie::ExtendedPose MEKF::get_state() const
{
    return lie::ExtendedPose{get_rot(), get_vel(), get_pos()};
}

void MEKF::set_state(const lie::ExtendedPose& state)
{
    m_x.segment<3>(kPosIndex) = state.position();
    m_x.segment<3>(kVelIndex) = state.velocity();
    m_x.segment<3>(kRotIndex) = Vector3::Zero();
    m_R_ref = state.rotation();
}

void MEKF::predict(double dt, const Vector3& acc, const Vector3& ang_vel, const ImuModel& imu_model)
{
    m_x = MEKF::state_transition_model(m_x, m_R_ref, dt, acc, ang_vel);

    // MEKF's error jacobian is different from IEKF's, so it is not computed by ImuModel.
    const auto& A = process_error_jacobian(m_R_ref, acc, ang_vel);
    const auto& Q_hat = imu_model.modified_noise_covariance();

    const Matrix<9,9> Phi = Matrix<9,9>::Identity() + A*dt; // Approximates Phi = exp(A*dt)
    m_P = Phi * (m_P + Q_hat*dt*dt) * Phi.transpose();

    reset_attitude_error();
}

double MEKF::update(const lie::Pose&, const MocapModel&)
{
    return 0.0;
}

double MEKF::update(const lie::Euclidean<3>& y, const GpsModel& sensor_model)
{
    const auto& H = sensor_model.error_jacobian();
    const auto& N_hat = sensor_model.modified_noise_covariance();

    const Matrix<3,3> Sinv = (H * m_P * H.transpose() + N_hat).inverse();
    const Matrix<9,3> K = m_P * H.transpose() * Sinv;

    const Vector3 innovation = y.vector() - sensor_model.h(get_state());

    m_x = m_x + K*innovation;
    m_P = (Covariance<9>::Identity() - K*H) * m_P;

    reset_attitude_error();

    return innovation.transpose() * Sinv * innovation;
}

void MEKF::reset_attitude_error()
{
    const Vector3 delta = m_x.segment<3>(kRotIndex);

    Matrix<9,9> T = Matrix<9,9>::Zero();
    T.block<3,3>(kPosIndex,kPosIndex) = Matrix3::Identity();
    T.block<3,3>(kVelIndex,kVelIndex) = Matrix3::Identity();
    T.block<3,3>(kRotIndex,kRotIndex) = lie::SO3::exp(-0.5 * delta).matrix();

    m_P = T * m_P * T.transpose();
    m_R_ref *= lie::SO3::exp(delta);
    m_x.segment<3>(kRotIndex) = Vector3::Zero();
}

// State transition model without resetting attitude error.
MEKF::State MEKF::state_transition_model(const State& x, const lie::Rotation& R_ref, double dt, const Vector3& acc, const Vector3& ang_vel)
{
    const Vector3 delta = x.segment<3>(kRotIndex);
    const Vector3 vel = x.segment<3>(kVelIndex);
    const Vector3 pos = x.segment<3>(kPosIndex);

    // const lie::Rotation R = R_ref * lie::SO3::exp(delta);               // Without inversion of R
    const lie::Rotation R = (R_ref * lie::SO3::exp(delta)).inverse();   // With inversion of R

    const Vector3 delta_pred = delta + (ang_vel - 0.5*lie::skew(ang_vel)*delta)*dt;
    const Vector3 vel_pred   = vel   + (R*acc + s_gravity)*dt;
    const Vector3 pos_pred   = pos   + vel*dt + (R*acc + s_gravity)*dt*dt*0.5;

    State x_pred;
    x_pred.segment<3>(kRotIndex) = delta_pred;
    x_pred.segment<3>(kVelIndex) = vel_pred;
    x_pred.segment<3>(kPosIndex) = pos_pred;

    return x_pred;
}

MEKF::Jacobian<9,9> MEKF::process_error_jacobian(const lie::Rotation& R_ref, const Vector3& acc, const Vector3& ang_vel)
{
    Jacobian<9,9> jac = Jacobian<9,9>::Zero();
    jac.block<3,3>(kPosIndex,kVelIndex) = Matrix3::Identity();
    // jac.block<3,3>(kVelIndex,kRotIndex) = lie::skew(R_ref*acc);                  // Without inversion of R
    jac.block<3,3>(kVelIndex,kRotIndex) = lie::skew(R_ref.inverse()*acc);        // With inversion of R
    jac.block<3,3>(kRotIndex,kRotIndex) = -0.5 * lie::skew(ang_vel);
    return jac;
}

} // namespace invariant
