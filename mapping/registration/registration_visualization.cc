#include "mapping/registration/registration_visualization.h"

#include <cmath>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <string>
#include <system_error>
#include <vector>

#include <Eigen/Core>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <pcl/filters/voxel_grid.h>

namespace adlabel {
namespace mapping {
namespace {

PointCloudXYZIRT VoxelGridDownsampleForVisualization(
    const PointCloudXYZIRT& cloud, double downsample_size) {
  PointCloudXYZIRT downsampled;
  pcl::VoxelGrid<PointXYZIRT> voxel_grid;
  voxel_grid.setInputCloud(cloud.makeShared());
  const float leaf_size = static_cast<float>(downsample_size);
  voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
  voxel_grid.filter(downsampled);
  return downsampled;
}

void DrawPoint(const Eigen::Vector3d& point, const cv::Vec3b& color,
               const RegistrationVisualizationOptions& options,
               cv::Mat* image) {
  const int center = options.image_size / 2;
  const int row =
      static_cast<int>(std::lround(center - point.x() / options.resolution));
  const int col =
      static_cast<int>(std::lround(center - point.y() / options.resolution));

  for (int row_offset = -options.point_radius;
       row_offset <= options.point_radius; ++row_offset) {
    for (int col_offset = -options.point_radius;
         col_offset <= options.point_radius; ++col_offset) {
      if (row_offset * row_offset + col_offset * col_offset >
          options.point_radius * options.point_radius) {
        continue;
      }

      const int pixel_row = row + row_offset;
      const int pixel_col = col + col_offset;
      if (pixel_row < 0 || pixel_row >= image->rows || pixel_col < 0 ||
          pixel_col >= image->cols) {
        continue;
      }
      image->at<cv::Vec3b>(pixel_row, pixel_col) = color;
    }
  }
}

void DrawCloud(const PointCloudXYZIRT& cloud,
               const Eigen::Isometry3d& target_from_cloud,
               const cv::Vec3b& color,
               const RegistrationVisualizationOptions& options,
               cv::Mat* image) {
  for (const auto& point : cloud.points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      continue;
    }
    DrawPoint(target_from_cloud * Eigen::Vector3d(point.x, point.y, point.z),
              color, options, image);
  }
}

cv::Mat RenderTopdownImage(const PointCloudXYZIRT& target,
                           const PointCloudXYZIRT& source,
                           const Eigen::Isometry3d& target_from_source,
                           const RegistrationVisualizationOptions& options) {
  cv::Mat image =
      cv::Mat::zeros(options.image_size, options.image_size, CV_8UC3);
  DrawCloud(target, Eigen::Isometry3d::Identity(), cv::Vec3b(0, 255, 0),
            options, &image);
  DrawCloud(source, target_from_source, cv::Vec3b(0, 0, 255), options, &image);
  return image;
}

void DrawRegistrationStats(const SmallGicpRegistrationResult& result,
                           cv::Mat* image) {
  std::ostringstream inlier_ratio;
  inlier_ratio << std::fixed << std::setprecision(2)
               << result.inlier_ratio * 100.0 << "%";

  const std::vector<std::string> lines = {
      std::string("converged: ") + (result.converged ? "true" : "false"),
      "iterations: " + std::to_string(result.iterations),
      "inlier_ratio: " + inlier_ratio.str(),
  };

  constexpr double kFontScale = 0.7;
  constexpr int kTextThickness = 2;
  constexpr int kOutlineThickness = 4;
  constexpr int kLineHeight = 32;
  constexpr int kLeftMargin = 20;
  constexpr int kTopMargin = 35;
  for (std::size_t i = 0; i < lines.size(); ++i) {
    const cv::Point origin(kLeftMargin,
                           kTopMargin + static_cast<int>(i) * kLineHeight);
    cv::putText(*image, lines[i], origin, cv::FONT_HERSHEY_SIMPLEX, kFontScale,
                cv::Scalar(0, 0, 0), kOutlineThickness, cv::LINE_AA);
    cv::putText(*image, lines[i], origin, cv::FONT_HERSHEY_SIMPLEX, kFontScale,
                cv::Scalar(255, 255, 255), kTextThickness, cv::LINE_AA);
  }
}

}  // namespace

RegistrationTopdownImages RenderRegistrationTopdownImages(
    const PointCloudXYZIRT& target, const PointCloudXYZIRT& source,
    const Eigen::Isometry3d& initial_target_source,
    const SmallGicpRegistrationResult& result,
    const RegistrationVisualizationOptions& options) {
  if (!std::isfinite(options.downsample_size) ||
      options.downsample_size < 0.0 || !std::isfinite(options.resolution) ||
      options.resolution <= 0.0 || options.image_size <= 0 ||
      options.point_radius < 0) {
    return {};
  }

  const PointCloudXYZIRT* visualization_target = &target;
  const PointCloudXYZIRT* visualization_source = &source;
  PointCloudXYZIRT downsampled_target;
  PointCloudXYZIRT downsampled_source;

  if (options.downsample_size > 0.0) {
    downsampled_target =
        VoxelGridDownsampleForVisualization(target, options.downsample_size);
    downsampled_source =
        VoxelGridDownsampleForVisualization(source, options.downsample_size);
    visualization_target = &downsampled_target;
    visualization_source = &downsampled_source;
  }

  RegistrationTopdownImages images;
  images.initial =
      RenderTopdownImage(*visualization_target, *visualization_source,
                         initial_target_source, options);
  images.optimized =
      RenderTopdownImage(*visualization_target, *visualization_source,
                         result.T_target_source, options);
  DrawRegistrationStats(result, &images.optimized);
  return images;
}

bool SaveRegistrationTopdownImages(const RegistrationTopdownImages& images,
                                   const std::string& output_dir) {
  if (images.initial.empty() || images.optimized.empty() ||
      images.initial.rows != images.optimized.rows ||
      images.initial.type() != images.optimized.type()) {
    return false;
  }

  const std::filesystem::path output_path(output_dir);
  std::error_code error;
  std::filesystem::create_directories(output_path, error);
  if (error) {
    return false;
  }

  cv::Mat comparison;
  cv::hconcat(images.initial, images.optimized, comparison);

  const bool initial_saved = cv::imwrite(
      (output_path / "registration_initial.png").string(), images.initial);
  const bool optimized_saved = cv::imwrite(
      (output_path / "registration_optimized.png").string(), images.optimized);
  const bool comparison_saved = cv::imwrite(
      (output_path / "registration_comparison.png").string(), comparison);
  return initial_saved && optimized_saved && comparison_saved;
}

}  // namespace mapping
}  // namespace adlabel
