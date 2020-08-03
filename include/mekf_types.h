#pragma once

#include <ugl/math/vector.h>
#include <ugl/math/matrix.h>

namespace mekf
{

using State              = ugl::Vector<9>;
using Covariance         = ugl::Matrix<9, 9>;
using Jacobian           = ugl::Matrix<9, 9>;

using Position           = ugl::Vector<3>;
using PositionCovariance = ugl::Matrix<3, 3>;
using PositionJacobian   = ugl::Matrix<3, 9>;

}