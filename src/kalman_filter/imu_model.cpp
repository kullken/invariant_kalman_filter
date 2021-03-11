#include "imu_model.h"

#include <ugl/math/matrix.h>
#include <ugl/lie_group/rotation.h>

namespace invariant
{

ImuModel::ImuModel(const ugl::Matrix<6,6>& noise_covariance)
    : m_modified_noise_covariance()
{
    static const ugl::Matrix<9,6> noise_jacobian = []() {
        ugl::Matrix<9,6> jac = ugl::Matrix<9,6>::Zero();
        jac.block<3,3>(0,0) = ugl::Matrix3::Identity();
        jac.block<3,3>(3,3) = ugl::Matrix3::Identity();
        return jac;
    }();
    m_modified_noise_covariance = noise_jacobian * noise_covariance * noise_jacobian.transpose();
}

ugl::Matrix<9,9> ImuModel::error_jacobian(const ugl::Vector3& acc, const ugl::Vector3& ang_vel) const
{
    ugl::Matrix<9,9> jacobian = ugl::Matrix<9,9>::Zero();
    jacobian.block<3,3>(0,0) = -ugl::lie::skew(ang_vel);
    jacobian.block<3,3>(3,3) = -ugl::lie::skew(ang_vel);
    jacobian.block<3,3>(6,6) = -ugl::lie::skew(ang_vel);
    jacobian.block<3,3>(3,0) = -ugl::lie::skew(acc);
    jacobian.block<3,3>(6,3) = ugl::Matrix3::Identity();
    return jacobian;
}

const ugl::Matrix<9,9>& ImuModel::modified_noise_covariance() const
{
    return m_modified_noise_covariance;
}

} // namespace invariant
