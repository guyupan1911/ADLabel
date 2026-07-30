#include "mapping/surfel_map/surfel_map.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Eigenvalues>
#include <glog/logging.h>

namespace adlabel {
namespace mapping {
namespace {

size_t MixHash(size_t seed, int32_t value) {
  const size_t hash = std::hash<int32_t>{}(value);
  return seed ^ (hash + 0x9e3779b9U + (seed << 6) + (seed >> 2));
}

bool IsFinite(const Eigen::Vector3d& point) {
  return point.array().isFinite().all();
}

}  // namespace

size_t VoxelKeyHash::operator()(const VoxelKey& key) const noexcept {
  size_t seed = 0;
  seed = MixHash(seed, key.x);
  seed = MixHash(seed, key.y);
  seed = MixHash(seed, key.z);
  return seed;
}

SurfelMap::SurfelMap(const SurfelMapOptions& options) : options_(options) {
  CHECK_GT(options_.voxel_resolution, 0.0);
  CHECK_GT(options_.texture_grid_size, 0);
  CHECK_GE(options_.minimum_point_count, 3u);
  CHECK_GT(options_.maximum_camera_candidates, 0u);
}

void SurfelMap::InsertGeometryPoint(const Eigen::Vector3d& point_map) {
  CHECK(!geometry_finalized_)
      << "cannot insert geometry after FinalizeGeometry";
  if (!IsFinite(point_map)) {
    return;
  }

  SurfelNode& surfel = surfels_[GetVoxelKey(point_map)];
  ++surfel.point_count;
  surfel.point_sum += point_map;
  surfel.point_outer_sum += point_map * point_map.transpose();
}

bool SurfelMap::FinalizeGeometry() {
  CHECK(!geometry_finalized_) << "FinalizeGeometry can only be called once";
  geometry_finalized_ = true;

  const int texture_cell_count =
      options_.texture_grid_size * options_.texture_grid_size;
  geometry_statistics_ = SurfelGeometryStatistics();
  geometry_statistics_.total_voxels = surfels_.size();

  for (auto& [key, surfel] : surfels_) {
    if (surfel.point_count < options_.minimum_point_count) {
      surfel.invalid_reason = SurfelInvalidReason::kInsufficientPoints;
      ++geometry_statistics_.insufficient_points;
      VLOG(1) << "filtered surfel voxel=(" << key.x << ", " << key.y << ", "
              << key.z
              << "): insufficient points, count=" << surfel.point_count;
      continue;
    }

    const double inverse_count = 1.0 / surfel.point_count;
    surfel.center = surfel.point_sum * inverse_count;
    Eigen::Matrix3d covariance = surfel.point_outer_sum * inverse_count -
                                 surfel.center * surfel.center.transpose();
    covariance = 0.5 * (covariance + covariance.transpose());

    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    if (solver.info() != Eigen::Success) {
      surfel.invalid_reason = SurfelInvalidReason::kEigenDecompositionFailure;
      ++geometry_statistics_.eigen_decomposition_failures;
      VLOG(1) << "filtered surfel voxel=(" << key.x << ", " << key.y << ", "
              << key.z << "): eigen decomposition failure";
      continue;
    }

    const double minimum_eigenvalue = std::max(0.0, solver.eigenvalues()(0));
    surfel.normal = solver.eigenvectors().col(0).normalized();
    if (surfel.normal.z() < 0.0) {
      surfel.normal = -surfel.normal;
    }
    surfel.plane_rms = static_cast<float>(std::sqrt(minimum_eigenvalue));

    surfel.geometry_valid = true;
    surfel.invalid_reason = SurfelInvalidReason::kNone;
    surfel.texture_cells.resize(texture_cell_count);
    ++geometry_statistics_.valid_surfels;
  }

  LOG(INFO) << "finalized surfel geometry: total_voxels="
            << geometry_statistics_.total_voxels
            << ", valid=" << geometry_statistics_.valid_surfels
            << ", invalid=" << geometry_statistics_.InvalidSurfelCount()
            << " [insufficient_points="
            << geometry_statistics_.insufficient_points << ", eigen_failure="
            << geometry_statistics_.eigen_decomposition_failures << "]";
  return geometry_statistics_.valid_surfels > 0;
}

VoxelKey SurfelMap::GetVoxelKey(const Eigen::Vector3d& point_map) const {
  return {
      static_cast<int32_t>(
          std::floor(point_map.x() / options_.voxel_resolution)),
      static_cast<int32_t>(
          std::floor(point_map.y() / options_.voxel_resolution)),
      static_cast<int32_t>(
          std::floor(point_map.z() / options_.voxel_resolution)),
  };
}

bool SurfelMap::AddCameraCandidate(const Eigen::Vector3d& support_point_map,
                                   const CameraCandidate& candidate) {
  CHECK(geometry_finalized_);
  SurfelNode* surfel = FindSurfel(GetVoxelKey(support_point_map));
  if (surfel == nullptr || !surfel->geometry_valid) {
    return false;
  }

  for (CameraCandidate& current : surfel->camera_candidates) {
    if (current.camera_frame_index == candidate.camera_frame_index) {
      current.support_score =
          std::max(current.support_score, candidate.support_score);
      return true;
    }
  }

  if (surfel->camera_candidates.size() < options_.maximum_camera_candidates) {
    surfel->camera_candidates.push_back(candidate);
    return true;
  }

  auto minimum = std::min_element(
      surfel->camera_candidates.begin(), surfel->camera_candidates.end(),
      [](const CameraCandidate& left, const CameraCandidate& right) {
        return left.support_score < right.support_score;
      });
  if (minimum != surfel->camera_candidates.end() &&
      candidate.support_score > minimum->support_score) {
    *minimum = candidate;
  }
  return true;
}

SurfelNode* SurfelMap::FindSurfel(const VoxelKey& key) {
  const auto iter = surfels_.find(key);
  return iter == surfels_.end() ? nullptr : &iter->second;
}

const SurfelNode* SurfelMap::FindSurfel(const VoxelKey& key) const {
  const auto iter = surfels_.find(key);
  return iter == surfels_.end() ? nullptr : &iter->second;
}

Eigen::Vector3d SurfelMap::GetTextureCellCenter(const VoxelKey& key,
                                                const SurfelNode& surfel,
                                                int cell_index) const {
  CHECK(surfel.geometry_valid);
  CHECK_GE(cell_index, 0);
  CHECK_LT(cell_index, options_.texture_grid_size * options_.texture_grid_size);

  const int cell_x = cell_index % options_.texture_grid_size;
  const int cell_y = cell_index / options_.texture_grid_size;
  const double cell_size =
      options_.voxel_resolution / options_.texture_grid_size;
  const double x =
      key.x * options_.voxel_resolution + (cell_x + 0.5) * cell_size;
  const double y =
      key.y * options_.voxel_resolution + (cell_y + 0.5) * cell_size;
  const double z =
      surfel.center.z() - (surfel.normal.x() * (x - surfel.center.x()) +
                           surfel.normal.y() * (y - surfel.center.y())) /
                              surfel.normal.z();
  return {x, y, z};
}

bool SurfelMap::UpdateTextureCell(const VoxelKey& key, int cell_index,
                                  const cv::Vec3b& bgr, float score,
                                  uint32_t camera_frame_index) {
  SurfelNode* surfel = FindSurfel(key);
  if (surfel == nullptr || !surfel->geometry_valid || cell_index < 0 ||
      cell_index >= static_cast<int>(surfel->texture_cells.size())) {
    return false;
  }

  SurfelTextureCell& cell = surfel->texture_cells[cell_index];
  if (!cell.valid || score > cell.score) {
    cell.bgr = bgr;
    cell.score = score;
    cell.source_camera_frame_index = camera_frame_index;
    cell.valid = true;
  }
  return true;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr SurfelMap::GetColoredPointCloud() const {
  auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  cloud->reserve(ColoredCellCount());

  for (const auto& [key, surfel] : surfels_) {
    if (!surfel.geometry_valid) {
      continue;
    }
    for (int i = 0; i < static_cast<int>(surfel.texture_cells.size()); ++i) {
      const SurfelTextureCell& cell = surfel.texture_cells[i];
      if (!cell.valid) {
        continue;
      }

      const Eigen::Vector3d position = GetTextureCellCenter(key, surfel, i);
      pcl::PointXYZRGB point;
      point.x = static_cast<float>(position.x());
      point.y = static_cast<float>(position.y());
      point.z = static_cast<float>(position.z());
      point.r = cell.bgr[2];
      point.g = cell.bgr[1];
      point.b = cell.bgr[0];
      cloud->push_back(point);
    }
  }

  cloud->width = static_cast<uint32_t>(cloud->size());
  cloud->height = 1;
  cloud->is_dense = true;
  return cloud;
}

bool SurfelMap::RenderBev(const GridFrame& frame, cv::Mat* rgb,
                          cv::Mat* coverage) const {
  CHECK(rgb != nullptr);
  CHECK(coverage != nullptr);
  if (frame.rows == 0 || frame.cols == 0 || frame.resolution <= 0.0) {
    return false;
  }

  *rgb = cv::Mat::zeros(static_cast<int>(frame.rows),
                        static_cast<int>(frame.cols), CV_8UC3);
  *coverage = cv::Mat::zeros(static_cast<int>(frame.rows),
                             static_cast<int>(frame.cols), CV_8UC1);
  cv::Mat best_score(static_cast<int>(frame.rows), static_cast<int>(frame.cols),
                     CV_32FC1,
                     cv::Scalar(std::numeric_limits<float>::lowest()));

  for (const auto& [key, surfel] : surfels_) {
    if (!surfel.geometry_valid) {
      continue;
    }
    for (int i = 0; i < static_cast<int>(surfel.texture_cells.size()); ++i) {
      const SurfelTextureCell& cell = surfel.texture_cells[i];
      if (!cell.valid) {
        continue;
      }

      const Eigen::Vector3d position = GetTextureCellCenter(key, surfel, i);
      unsigned int row = 0;
      unsigned int col = 0;
      if (!frame.WorldToPixel(position.head<2>(), &row, &col)) {
        continue;
      }

      float& current_score =
          best_score.at<float>(static_cast<int>(row), static_cast<int>(col));
      if (cell.score <= current_score) {
        continue;
      }
      current_score = cell.score;
      rgb->at<cv::Vec3b>(static_cast<int>(row), static_cast<int>(col)) =
          cell.bgr;
      coverage->at<uint8_t>(static_cast<int>(row), static_cast<int>(col)) = 255;
    }
  }
  return true;
}

size_t SurfelMap::ValidSurfelCount() const {
  size_t count = 0;
  for (const auto& [key, surfel] : surfels_) {
    if (surfel.geometry_valid) {
      ++count;
    }
  }
  return count;
}

size_t SurfelMap::ColoredCellCount() const {
  size_t count = 0;
  for (const auto& [key, surfel] : surfels_) {
    for (const SurfelTextureCell& cell : surfel.texture_cells) {
      if (cell.valid) {
        ++count;
      }
    }
  }
  return count;
}

}  // namespace mapping
}  // namespace adlabel
