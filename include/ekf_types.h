#pragma once

#include <ugl/math/vector.h>
#include <ugl/math/matrix.h>
#include <ugl/math/quaternion.h>

namespace ekf
{

using namespace ugl::math;

using State              = Vector<9>;
using Covariance         = Matrix<9, 9>;
using Jacobian           = Matrix<9, 9>;

using Position           = Vector<3>;
using PositionCovariance = Matrix<3, 3>;
using PositionJacobian   = Matrix<3, 9>;

}