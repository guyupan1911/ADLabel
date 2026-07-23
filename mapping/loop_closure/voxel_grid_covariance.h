#pragma once

// FLANN's LSH table can use std::unordered_map when USE_UNORDERED_MAP is
// enabled, but FLANN's own serialization helpers only support std::map in
// this code path. Include FLANN's config first, then force the std::map
// branch and include PCL's FLANN-backed kdtree before any OpenCV FLANN
// header can redefine USE_UNORDERED_MAP.
#include <flann/config.h>

#ifdef USE_UNORDERED_MAP
#undef USE_UNORDERED_MAP
#endif
#define USE_UNORDERED_MAP 0

#include <pcl/filters/voxel_grid_covariance.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace adlabel {
namespace mapping {

class VoxelGridCovariance : public pcl::VoxelGridCovariance<pcl::PointXYZ> {
 public:
  void setCovEigValueInflationRatio(const double min_covar_eigvalue_mult) {
    min_covar_eigvalue_mult_ = min_covar_eigvalue_mult;
  }
};

}  // namespace mapping
}  // namespace adlabel
