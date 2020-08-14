#include "iekf.h"

#include <ugl/math/vector.h>
#include <ugl/math/matrix.h>

#include <ugl/lie_group/rotation.h>
#include <ugl/lie_group/pose.h>
#include <ugl/lie_group/extended_pose.h>

#include "iekf_types.h"

namespace invariant
{

using ugl::Vector3;
using ugl::Matrix3;
using ugl::lie::Rotation;

using ugl::Vector;
using ugl::Matrix;

const Vector3 IEKF::s_gravity{0.0, 0.0, -9.82};

IEKF::IEKF(const Rotation& R0, const Vector3& p0, const Vector3& v0, const Covariance<9>& P0)
    : m_X(R0, v0, p0)
    , m_P(P0)
{
}

void IEKF::predict(double dt, const Vector3& acc, const Vector3& ang_vel)
{
    // State propagation
    const Rotation R = m_X.rotation();
    const Vector3  v = m_X.velocity();
    const Vector3  p = m_X.position();

    // Discretisation method from Hartley et al. (2018)
    const Rotation R_pred = R * ugl::lie::SO3::exp(ang_vel*dt);
    const Vector3  v_pred = v + (R*acc + s_gravity)*dt;
    const Vector3  p_pred = p + v*dt + 0.5*(R*acc + s_gravity)*dt*dt;

    m_X = ugl::lie::ExtendedPose{R_pred, v_pred, p_pred};

    // Error propagation
    Matrix<9,9> A = Matrix<9,9>::Zero();
    A.block<3,3>(0,0) = ugl::lie::skew(ang_vel).transpose();
    A.block<3,3>(3,3) = ugl::lie::skew(ang_vel).transpose();
    A.block<3,3>(6,6) = ugl::lie::skew(ang_vel).transpose();
    A.block<3,3>(3,0) = ugl::lie::skew(acc).transpose();
    A.block<3,3>(6,3) = Matrix3::Identity();

    const Covariance<9> Q = Matrix<9,9>::Identity() * 0.1;    // TODO: Set to something smart.
    const Matrix<9,9> D = Matrix<9,9>::Identity();            // TODO: This is only here to make the algorithm clearer in the code.

    // Discretisation method from Hartley et al. (2018)
    // const Matrix<9,9> Phi = ugl::math::exp(Matrix<9,9>(A*dt));
    const Matrix<9,9> Adt = A*dt;
    const Matrix<9,9> Adt2 = Adt*Adt;
    const Matrix<9,9> Phi = Matrix<9,9>::Identity() + Adt + Adt2/2 + Adt2*Adt/6 + Adt2*Adt2/24; // Approximates exp(A*dt)
    m_P = Phi*m_P*Phi.transpose() + Phi*D*Q*D.transpose()*Phi.transpose();
}

void IEKF::mocap_update(const Rotation& R_measured, const Vector3& pos_measured)
{
    // Build measuremnt variable.
    Matrix<5,4> Y = Matrix<5,4>::Zero();
    Y.block<3,3>(0,0) = R_measured.matrix();
    Y.block<3,1>(0,3) = pos_measured;
    Y(4,3) = 1.0;

    // Measurement noise of mocap.
    const Covariance<6> N = Covariance<6>::Identity() * 0.01; // TODO: Set to something smart.

    // Gain calculation
    Matrix<6,9> H = Matrix<6,9>::Zero();
    H.block<3,3>(0,0) = Matrix3::Identity();
    H.block<3,3>(3,6) = Matrix3::Identity();

    const Matrix<6,6> S = H * m_P * H.transpose() + N;
    const Matrix<9,6> L = m_P * H.transpose() * S.inverse();

    Matrix<5,4> D = Matrix<5,4>::Zero();
    D.block<3,3>(0,0) = Matrix3::Identity();
    D(4,3) = 1.0;
    const Matrix<4,5> D_left_pseudo_inv = (D.transpose() * D).inverse() * D.transpose();

    // State correction
    const Vector<6> innovation = ugl::lie::SE_3::log(ugl::lie::SE_3{D_left_pseudo_inv * m_X.inverse().matrix() * Y});
    const ugl::lie::ExtendedPose dX = ugl::lie::SE2_3::exp(L*innovation);
    m_X = m_X*dX;

    // Error correction
    m_P = (Covariance<9>::Identity() - L*H) * m_P;     // TODO: Inplace subtraction might be faster (m_P -= L*H*m_P). Test if works with Eigen.
}

}