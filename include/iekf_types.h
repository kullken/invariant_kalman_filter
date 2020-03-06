#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace iekf
{

template<int rows>
using Vector = Eigen::Matrix<double, rows, 1>;

template<int rows, int cols>
using Matrix = Eigen::Matrix<double, rows, cols>;

template<int n>
using Covariance = Eigen::Matrix<double, n, n>;

using Vector3    = Eigen::Matrix<double, 3, 1>;
using Matrix3    = Eigen::Matrix<double, 3, 3>;

using Rotation   = Eigen::Matrix<double, 3, 3>;
using Quaternion = Eigen::Quaternion<double>;

using Jacobian   = Eigen::Matrix<double, 9, 9>;


class State : public Matrix<5, 5>
{
public:
    State() : Matrix(Matrix::Identity()) {}
    
    template<typename Derived>
    State(const Eigen::MatrixBase<Derived>& other) : Matrix(other) {}
    
    template<typename Derived>
    State& operator=(const Eigen::MatrixBase <Derived>& other)
    {
        this->Matrix::operator=(other);
        return *this;
    }

    // TODO: Should be possible to return references/views instead of copy. #EigenProblems
    Rotation get_rot() const { return this->block<3,3>(0, 0); }
    Vector3  get_pos() const { return this->block<3,1>(0, 3); }
    Vector3  get_vel() const { return this->block<3,1>(0, 4); }
    // const Rotation& get_rot() const { return this->block<3,3>(0, 0); }
    // const Vector3&  get_pos() const { return this->block<3,1>(0, 3); }
    // const Vector3&  get_vel() const { return this->block<3,1>(0, 4); }

    void set_rot(const Rotation& rot) { this->block<3,3>(0, 0) = rot; }
    void set_pos(const Vector3& pos) { this->block<3,1>(0, 3) = pos; }
    void set_vel(const Vector3& vel) { this->block<3,1>(0, 4) = vel; }
};

}