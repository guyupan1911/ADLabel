#include "mapping/mapping_utils/geographic_transform.h"

#include <cmath>

#include <GeographicLib/Geocentric.hpp>

namespace adlabel {
namespace mapping {

Eigen::Affine3d MakeEcefToEnuTransform(const Eigen::Vector3d& origin_ecef) {
  double latitude_deg = 0.0;
  double longitude_deg = 0.0;
  double height = 0.0;
  GeographicLib::Geocentric::WGS84().Reverse(origin_ecef.x(), origin_ecef.y(),
                                             origin_ecef.z(), latitude_deg,
                                             longitude_deg, height);

  constexpr double kDegreesToRadians = 3.14159265358979323846 / 180.0;
  const double latitude = latitude_deg * kDegreesToRadians;
  const double longitude = longitude_deg * kDegreesToRadians;
  const double sin_latitude = std::sin(latitude);
  const double cos_latitude = std::cos(latitude);
  const double sin_longitude = std::sin(longitude);
  const double cos_longitude = std::cos(longitude);

  Eigen::Matrix3d ecef_to_enu_rotation;
  ecef_to_enu_rotation << -sin_longitude, cos_longitude, 0.0,
      -sin_latitude * cos_longitude, -sin_latitude * sin_longitude,
      cos_latitude, cos_latitude * cos_longitude, cos_latitude * sin_longitude,
      sin_latitude;

  Eigen::Affine3d T_enu_ecef = Eigen::Affine3d::Identity();
  T_enu_ecef.linear() = ecef_to_enu_rotation;
  T_enu_ecef.translation() = -ecef_to_enu_rotation * origin_ecef;
  return T_enu_ecef;
}

}  // namespace mapping
}  // namespace adlabel
