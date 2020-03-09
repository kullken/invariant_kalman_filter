#include "iekf.h"

// #include <ros/console.h>

#include "iekf_types.h"
#include "lie_group.h"

namespace invariant
{

const Vector3 IEKF::s_gravity{0.0, 0.0, -9.82};

IEKF::IEKF(const Rotation& R0, const Vector3& p0, const Vector3& v0, const Covariance<9>& P0)
    : m_X()
    , m_P(P0)
{
    m_X.set_rot(R0);
    m_X.set_pos(p0);
    m_X.set_vel(v0);
}

void IEKF::predict(double dt, const Vector3& acc, const Vector3& ang_vel)
{
    // State propagation
    const Rotation R = m_X.get_rot();
    const Vector3  p = m_X.get_pos();
    const Vector3  v = m_X.get_vel();

    // Discretisation method from Hartley et al. (2018)
    const Rotation R_pred = R * lie::exp_map_SO_3(ang_vel*dt);
    const Vector3  p_pred = p + v*dt + 0.5*(R*acc + s_gravity)*dt*dt;
    const Vector3  v_pred = v + (R*acc + s_gravity)*dt;

    m_X.set_rot(R_pred);
    m_X.set_pos(p_pred);
    m_X.set_vel(v_pred);

    // Error propagation
    Matrix<9,9> A = Matrix<9,9>::Zero();
    A.block<3,3>(0,0) = lie::skew(ang_vel).transpose();
    A.block<3,3>(3,3) = lie::skew(ang_vel).transpose();
    A.block<3,3>(6,6) = lie::skew(ang_vel).transpose();
    A.block<3,3>(6,0) = lie::skew(acc).transpose();
    A.block<3,3>(3,6) = Matrix3::Identity();

    Covariance<9> Q = Matrix<9,9>::Identity() * 0.1;    // TODO: Set to something smart.
    Matrix<9,9> D = Matrix<9,9>::Identity();            // TODO: This is only here to make the algorithm clearer in the code.

    // Discretisation method from Hartley et al. (2018)
    Matrix<9,9> Phi = lie::expm<9>(A*dt);           // TODO: Check if approximation I + (A*dt)^ is accurate enough and faster. 
    Covariance<9> P_pred = Phi*m_P*Phi.transpose() + Phi*D*Q*D.transpose()*Phi.transpose();

    m_P = P_pred; // TODO: Self-assignment would likely be faster. Was there some problem with Eigen and self-assignment?
}

void IEKF::mocap_update(const Rotation& R_measured, const Vector3& pos_measured)
{
    // Build measuremnt variable.
    Matrix<5,4> Y = Matrix<5,4>::Identity();
    Y.block<3,3>(0,0) = R_measured;
    Y.block<3,1>(0,3) = pos_measured;

    // Measurement noise of mocap.
    Covariance<6> N = Covariance<6>::Identity() * 0.01; // TODO: Set to something smart.

    // Gain calculation
    Matrix<6,9> H = Matrix<6,9>::Identity();   
    Matrix<6,6> S = H * m_P * H.transpose() + N; 
    Matrix<9,6> L = m_P * H.transpose() * S.inverse();

    // State correction
    Vector<6> innovation = lie::log_map_SE_3(Matrix<4,5>::Identity() * m_X.inverse() * Y);
    State dX = lie::exp_map_SE2_3(L*innovation);
    m_X = m_X*dX;

    // Error correction
    m_P = (Covariance<9>::Identity() - L*H) * m_P;     // TODO: Inplace subtraction might be faster (m_P -= L*H*m_P). Test if works with Eigen.
}

}