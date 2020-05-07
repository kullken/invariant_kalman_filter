#pragma once

#include <ugl/math/vector.h>
#include <ugl/math/matrix.h>
#include <ugl/math/quaternion.h>

namespace invariant
{

template<int n>
using Covariance = ugl::Matrix<n, n>;

using Jacobian   = ugl::Matrix<9, 9>;

class State : public ugl::Matrix<5, 5>
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
    ugl::Rotation get_rot() const { return this->block<3,3>(0, 0); }
    ugl::Vector3  get_pos() const { return this->block<3,1>(0, 3); }
    ugl::Vector3  get_vel() const { return this->block<3,1>(0, 4); }
    // const Rotation& get_rot() const { return this->block<3,3>(0, 0); }
    // const Vector3&  get_pos() const { return this->block<3,1>(0, 3); }
    // const Vector3&  get_vel() const { return this->block<3,1>(0, 4); }

    void set_rot(const ugl::Rotation& rot) { this->block<3,3>(0, 0) = rot; }
    void set_pos(const ugl::Vector3& pos) { this->block<3,1>(0, 3) = pos; }
    void set_vel(const ugl::Vector3& vel) { this->block<3,1>(0, 4) = vel; }
};

}