#pragma once

#include <unsupported/Eigen/MatrixFunctions>

#include "iekf_types.h"


namespace invariant::lie
{

template<int n>
Matrix<n,n> expm(const Matrix<n,n>& mat)
{
    return mat.exp();
}

template<int n>
Matrix<n,n> logm(const Matrix<n,n>& mat)
{
    return mat.log();
}

/// Skew-symmetric cross-product matrix of a 3D vector. 
/// Map: R^3 -> so(3).
Matrix3 skew(const Vector3& w);

/// Cross-product vector corresponding to skew-symmetric matrix.
/// Map: so(3) -> R^3.
Vector3 unskew(const Matrix3& S);

/// Map: R^3 -> SO(3).
Matrix3 exp_map_SO_3(const Vector3& w);

/// Map: SE_(3) -> R^9.
Vector<6> log_map_SE_3(const Matrix<4,4>& U);

/// Map: R^9 -> se_2(3).
Matrix<5,5> hat_map_se2_3(const Vector<9>& u);

/// Map: R^9 -> SE_2(3).
Matrix<5,5> exp_map_SE2_3(const Vector<9>& u);

/// Map: SE_2(3) -> R^9.
Vector<9> log_map_SE2_3(const Matrix<5,5>& U);

Matrix<9,9> Adjoint_SE2_3(const State& X);



}