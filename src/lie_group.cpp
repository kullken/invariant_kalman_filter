#include "lie_group.h"

#include <cassert>
#include <cmath>

#include <ros/console.h>

#include <unsupported/Eigen/MatrixFunctions>

#include "iekf_types.h"


namespace invariant::lie
{

Quaternion exp(const Quaternion& q)
{
    [[maybe_unused]] const double tol = 1e-6;
    assert(("Imaginary quaternion required.", q.w() < tol));
    Vector3 u = q.vec();
    double u_norm = u.norm();
    Vector3 v = u.normalized() * std::sin(u_norm);
    return Quaternion(std::cos(u_norm), v.x(), v.y(), v.z());
}

Quaternion log(const Quaternion& q)
{
    [[maybe_unused]] const double tol = 1e-6;
    assert(("Unit-norm quaternion required.", std::abs(q.norm() - 1) < tol));
    Vector3 u = q.vec();
    Vector3 v = u.normalized() * std::acos(q.w());
    return Quaternion(0, v.x(), v.y(), v.z());
}

Matrix3 skew(const Vector3& w)
{
    Matrix3 S;
    S << 0.0, -w(2), w(1),
         w(2), 0.0, -w(0),
        -w(1), w(0), 0.0;
    return S;
}

Vector3 unskew(const Matrix3& S)
{
    const double tol = 1e-6;
    double trace = S.trace();
    ROS_WARN_STREAM_COND(trace > tol, "Matrix to unskew has high trace: " << trace);
    ROS_WARN_STREAM_COND(S(2,1) + S(1,2) > tol, "Matrix to unskew is not skew-symmetric in w(0): " << S(2,1) + S(1,2));
    ROS_WARN_STREAM_COND(S(0,2) + S(2,0) > tol, "Matrix to unskew is not skew-symmetric in w(0): " << S(0,2) + S(2,0));
    ROS_WARN_STREAM_COND(S(1,0) + S(0,1) > tol, "Matrix to unskew is not skew-symmetric in w(0): " << S(1,0) + S(0,1));

    if (trace > tol || S(2,1) + S(1,2) > tol || S(0,2) + S(2,0)  > tol || S(1,0) + S(0,1) > tol)
    {
        ROS_WARN_STREAM("Matrix:\n" << S);
    }

    return Vector3(S(2,1)-S(1,2), S(0,2)-S(2,0), S(1,0)-S(0,1)) * 0.5;
}

Matrix3 exp_map_SO_3(const Vector3& w)
{
    // const double phi = w.norm();
    // const Matrix3 S = skew(w);
    // const Matrix3 I = Matrix3::Identity();

    // Matrix3 R = I + (std::sin(phi)/phi) * S + ((1 - std::cos(phi))/(phi*phi)) * S*S;
    // return R;
    return skew(w).exp();
}

Vector<6> log_map_SE_3(const Matrix<4,4>& U)
{
    Matrix<4,4> log_U = logm(U);
    Vector<6> u;
    u.segment<3>(0) = unskew(log_U.block<3,3>(0,0));
    u.segment<3>(3) = log_U.block<3,1>(0,3);
    return u;
}

Matrix<5,5> hat_map_se2_3(const Vector<9>& u)
{
    Matrix<5,5> mat = Matrix<5,5>::Zero();
    mat.block<3,3>(0,0) = skew(u.segment<3>(0));
    mat.block<3,1>(0,3) = u.segment<3>(3);
    mat.block<3,1>(0,4) = u.segment<3>(6);
    return mat;
}

/// Based on Barrau17a-Invariant_EKF_stable_observer and Barfoot17-State_Estimation_for_Robotics
Matrix<5,5> exp_map_SE2_3(const Vector<9>& zeta)
{
    const double phi = zeta.segment<3>(0).norm();
    const Matrix<5,5> S = hat_map_se2_3(zeta);
    const Matrix<5,5> I = Matrix<5,5>::Identity();

    return I + S + (1 - std::cos(phi))/(phi*phi) * S*S + (phi - std::sin(phi))/(phi*phi*phi) * S*S*S;
}

Vector<9> log_map_SE2_3(const Matrix<5,5>& U)
{
    Matrix<5,5> log_U = logm(U);
    Vector<9> u;
    u.segment<3>(0) = unskew(log_U.block<3,3>(0,0));
    u.segment<3>(3) = log_U.block<3,1>(0,3);
    u.segment<3>(6) = log_U.block<3,1>(0,4);
    return u;
}

Matrix<9,9> Adjoint_SE2_3(const State& X)
{
    const Rotation& R = X.get_rot();
    const Vector3&  p = X.get_pos();
    const Vector3&  v = X.get_vel();

    Matrix<9,9> Adj = Matrix<9,9>::Zero();
    Adj.block<3,3>(0,0) = R;
    Adj.block<3,3>(3,3) = R;
    Adj.block<3,3>(6,6) = R;
    Adj.block<3,3>(3,0) = skew(p) * R;
    Adj.block<3,3>(6,0) = skew(v) * R;

    return Adj;
}


}