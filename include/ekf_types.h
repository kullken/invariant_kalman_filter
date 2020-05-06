#pragma once

#include <geometry_cpp/math/vector.h>
#include <geometry_cpp/math/matrix.h>
#include <geometry_cpp/math/quaternion.h>

namespace ekf
{

using namespace geometry::math;

using State              = Vector<9>;
using Covariance         = Matrix<9, 9>;
using Jacobian           = Matrix<9, 9>;

using Position           = Vector<3>;
using PositionCovariance = Matrix<3, 3>;
using PositionJacobian   = Matrix<3, 9>;

}