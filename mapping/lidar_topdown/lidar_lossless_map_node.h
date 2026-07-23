#pragma once

#include <Eigen/Core>
#include <cstddef>
#include <cstdint>
#include <opencv2/core.hpp>
#include <string>

#include "mapping/lidar_topdown/grid_frame.h"
#include "mapping/lidar_topdown/lossless_map_matrix.h"

namespace adlabel {
namespace mapping {

// Sentinel for empty cells in the rescaled altitude image (CV_16UC1).
// Valid altitudes are mapped to [1, 65535].
constexpr uint16_t kEmptyAltitudeValue = 0;

class LidarLosslessMapNode {
 public:
  enum class IntensityMappingMode { kLogarithmic, kPassThrough };
  enum class IntensityAggregationMode { kMax, kMean };

  LidarLosslessMapNode() = default;
  ~LidarLosslessMapNode() = default;

  void Init(const GridFrame& frame,
            IntensityMappingMode intensity_mapping =
                IntensityMappingMode::kLogarithmic,
            IntensityAggregationMode intensity_aggregation =
                IntensityAggregationMode::kMax);
  void Reset();

  const GridFrame& GetFrame() const { return frame_; }
  void SetLeftTopCorner(double x, double y);

  bool SetValue(const Eigen::Vector3d& world_xyz, const std::string& lidar_name,
                unsigned char intensity);

  void GetIntensityImage(cv::Mat* image, size_t min_samples = 0) const;
  // Empty cells filled with NaN.
  void GetAltitudeImage(cv::Mat* image, size_t min_samples = 0) const;
  // Empty cells filled with kEmptyAltitudeValue.
  void GetRescaledAltitudeImage(float min_altitude, float max_altitude,
                                cv::Mat* image, size_t min_samples = 0) const;

  double GetOccupancyRatio() const;

  const LosslessMapMatrix& GetMatrix() const { return matrix_; }
  LosslessMapMatrix& GetMatrix() { return matrix_; }

 private:
  GridFrame frame_;
  IntensityMappingMode intensity_mapping_ = IntensityMappingMode::kLogarithmic;
  IntensityAggregationMode intensity_aggregation_ =
      IntensityAggregationMode::kMax;
  DenseLosslessMapMatrix matrix_;
};

}  // namespace mapping
}  // namespace adlabel
