#pragma once

#include <ugl/math/vector.h>
#include <ugl/math/matrix.h>

namespace mekf
{

template<int n>
using Covariance = ugl::Matrix<n, n>;

using State              = ugl::Vector<9>;
using Jacobian           = ugl::Matrix<9, 9>;

using Position           = ugl::Vector<3>;
using PositionJacobian   = ugl::Matrix<3, 9>;

}