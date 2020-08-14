#pragma once

#include <ugl/math/vector.h>
#include <ugl/math/matrix.h>
#include <ugl/math/quaternion.h>

namespace invariant
{

template<int n>
using Covariance = ugl::Matrix<n, n>;

using Jacobian   = ugl::Matrix<9, 9>;

}