#pragma once

#include <string>

#include <Eigen/Geometry>
#include <opencv2/core.hpp>

#include "mapping/common/pcl_types.h"
#include "mapping/registration/small_gicp_types.h"

namespace adlabel {
namespace mapping {

struct RegistrationTopdownImages {
  cv::Mat initial;
  cv::Mat optimized;
};

struct RegistrationVisualizationOptions {
  // Visualization-only voxel size in meters. Zero disables downsampling.
  double downsample_size = 0.5;

  // Topdown image resolution in meters per pixel.
  double resolution = 0.1;

  // Width and height of the square output image.
  int image_size = 1600;

  // Rendered point radius in pixels.
  int point_radius = 1;
};

// Renders target in green and source in red from a topdown view. Source points
// overwrite target points where they overlap. Both images use the same
// target-frame view. Registration statistics are drawn on the optimized image.
RegistrationTopdownImages RenderRegistrationTopdownImages(
    const PointCloudXYZIRT& target, const PointCloudXYZIRT& source,
    const Eigen::Isometry3d& initial_target_source,
    const SmallGicpRegistrationResult& result,
    const RegistrationVisualizationOptions& options =
        RegistrationVisualizationOptions());

// Creates output_dir and saves initial, optimized, and side-by-side comparison
// PNG images. Returns false if the images are invalid or cannot be written.
bool SaveRegistrationTopdownImages(const RegistrationTopdownImages& images,
                                   const std::string& output_dir);

}  // namespace mapping
}  // namespace adlabel
