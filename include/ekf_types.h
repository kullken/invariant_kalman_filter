#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace ekf
{

template<int rows, int cols>
using Matrix             = Eigen::Matrix<double, rows, cols>;

using Vec3               = Eigen::Matrix<double, 3, 1>;
using Matrix3            = Eigen::Matrix<double, 3, 3>;

using State              = Eigen::Matrix<double, 9, 1>;
using Covariance         = Eigen::Matrix<double, 9, 9>;
using Jacobian           = Eigen::Matrix<double, 9, 9>;

using Position           = Eigen::Matrix<double, 3, 1>;
using PositionCovariance = Eigen::Matrix<double, 3, 3>;
using PositionJacobian   = Eigen::Matrix<double, 3, 9>;

using Rotation           = Eigen::Matrix<double, 3, 3>;
using Quaternion         = Eigen::Quaternion<double>;

}