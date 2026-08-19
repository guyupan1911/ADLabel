#pragma once

#include <Eigen/Geometry>

namespace adlabel {
namespace mapping {

// Returns the transform from ECEF to a local ENU coordinate system whose
// origin is origin_ecef.
Eigen::Affine3d MakeEcefToEnuTransform(const Eigen::Vector3d& origin_ecef);

}  // namespace mapping
}  // namespace adlabel
