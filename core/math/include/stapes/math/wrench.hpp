#pragma once

#include <Eigen/Core>

namespace stapes::math {

/// Force (N, rows 0-2) and torque (N m, rows 3-5) acting at a point.
using Wrench = Eigen::Matrix<double, 6, 1>;

}  // namespace stapes::math
