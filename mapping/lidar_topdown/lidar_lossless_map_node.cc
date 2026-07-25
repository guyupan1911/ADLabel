#include "mapping/lidar_topdown/lidar_lossless_map_node.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include <opencv2/imgproc.hpp>

namespace adlabel {
namespace mapping {
namespace {

uint16_t GetRescaledAltitude(float alt, float min_alt, float max_alt) {
  if (max_alt <= min_alt) return kEmptyAltitudeValue + 1;
  const float t = (alt - min_alt) / (max_alt - min_alt);
  const float clamped = std::max(0.f, std::min(1.f, t));
  return static_cast<uint16_t>(1u + static_cast<unsigned>(clamped * 65534.f));
}

unsigned char ApplyIntensityMapping(
    float intensity, LidarLosslessMapNode::IntensityMappingMode mode) {
  if (mode == LidarLosslessMapNode::IntensityMappingMode::kPassThrough) {
    return static_cast<unsigned char>(intensity);
  }
  if (intensity <= 0.0f) return 0;
  const float val = std::log2(intensity) * 32.0f;
  return static_cast<unsigned char>(std::max(0.0f, std::min(255.0f, val)));
}

}  // namespace

void LidarLosslessMapNode::Init(const GridFrame& frame,
                                IntensityMappingMode intensity_mapping,
                                IntensityAggregationMode intensity_aggregation,
                                MatrixType matrix_type) {
  frame_ = frame;
  intensity_mapping_ = intensity_mapping;
  intensity_aggregation_ = intensity_aggregation;
  if (matrix_type == MatrixType::kDense) {
    matrix_ = std::make_unique<DenseLosslessMapMatrix>();
  } else {
    matrix_ = std::make_unique<SparseLosslessMapMatrix>();
  }
  matrix_->Init(frame.rows, frame.cols);
}

void LidarLosslessMapNode::Reset() { matrix_->Reset(); }

void LidarLosslessMapNode::SetLeftTopCorner(double x, double y) {
  frame_.top_left_corner = {x, y};
}

bool LidarLosslessMapNode::SetValue(const Eigen::Vector3d& world_xyz,
                                    const std::string& lidar_name,
                                    unsigned char intensity) {
  unsigned int row, col;
  if (!frame_.WorldToPixel({world_xyz.x(), world_xyz.y()}, &row, &col)) {
    return false;
  }

  LosslessMapCell& cell = matrix_->GetOrCreate(row, col);
  if (intensity_aggregation_ == IntensityAggregationMode::kMean) {
    cell.AddSampleMean(static_cast<float>(world_xyz.z()), intensity);
  } else {
    cell.AddSampleMax(static_cast<float>(world_xyz.z()), intensity);
  }
  return true;
}

void LidarLosslessMapNode::GetIntensityImage(cv::Mat* image,
                                             size_t min_samples) const {
  *image = cv::Mat::zeros(static_cast<int>(frame_.rows),
                          static_cast<int>(frame_.cols), CV_8UC1);
  matrix_->ForEachOccupied(
      [&](unsigned int r, unsigned int c, const LosslessMapCell& cell) {
        if (cell.GetCount() < min_samples) return;
        image->at<unsigned char>(static_cast<int>(r), static_cast<int>(c)) =
            ApplyIntensityMapping(cell.intensity, intensity_mapping_);
      });

  cv::normalize(*image, *image, 0, 255, cv::NORM_MINMAX);
  const cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
  clahe->apply(*image, *image);
}

void LidarLosslessMapNode::GetAltitudeImage(cv::Mat* image,
                                            size_t min_samples) const {
  *image =
      cv::Mat(static_cast<int>(frame_.rows), static_cast<int>(frame_.cols),
              CV_32FC1, cv::Scalar(std::numeric_limits<float>::quiet_NaN()));
  matrix_->ForEachOccupied([&](unsigned int r, unsigned int c,
                               const LosslessMapCell& cell) {
    if (cell.GetCount() < min_samples) return;
    image->at<float>(static_cast<int>(r), static_cast<int>(c)) = cell.GetAlt();
  });
}

void LidarLosslessMapNode::GetRescaledAltitudeImage(float min_altitude,
                                                    float max_altitude,
                                                    cv::Mat* image,
                                                    size_t min_samples) const {
  *image = cv::Mat(static_cast<int>(frame_.rows), static_cast<int>(frame_.cols),
                   CV_16UC1, cv::Scalar(kEmptyAltitudeValue));
  matrix_->ForEachOccupied(
      [&](unsigned int r, unsigned int c, const LosslessMapCell& cell) {
        if (cell.GetCount() < min_samples) return;
        image->at<uint16_t>(static_cast<int>(r), static_cast<int>(c)) =
            GetRescaledAltitude(cell.GetAlt(), min_altitude, max_altitude);
      });
}

double LidarLosslessMapNode::GetOccupancyRatio() const {
  const size_t total = static_cast<size_t>(frame_.rows) * frame_.cols;
  if (total == 0) return 0.0;
  size_t occupied = 0;
  matrix_->ForEachOccupied(
      [&](unsigned int, unsigned int, const LosslessMapCell&) { ++occupied; });
  return static_cast<double>(occupied) / static_cast<double>(total);
}

}  // namespace mapping
}  // namespace adlabel
