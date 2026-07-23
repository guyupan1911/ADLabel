#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "mapping/protos/camera_calibration.pb.h"

namespace adlabel {
namespace mapping {

// Camera models store intrinsics only. Extrinsics are provided by callers when
// transforming points into the camera coordinate system.
class BaseCamera {
 public:
  BaseCamera() = default;
  virtual ~BaseCamera() = default;

  CameraType Type() const { return type_; }
  virtual std::string ModelName() const = 0;

  size_t Width() const { return width_; }
  size_t Height() const { return height_; }

  double FocalLengthX() const { return params_[0]; }
  double FocalLengthY() const { return params_[1]; }
  double PrincipalPointX() const { return params_[2]; }
  double PrincipalPointY() const { return params_[3]; }

  const std::vector<double>& Params() const { return params_; }
  std::vector<double> DistortionCoefficients() const;

  Eigen::Matrix3d CalibrationMatrix() const;
  CameraCalibration ToProto() const;

  // Projects a point in camera coordinates. The camera frame uses z forward,
  // x right, y down. Returns false for points behind the camera or outside
  // the image.
  virtual bool Project(const Eigen::Vector3d& point_cam,
                       Eigen::Vector2d* pixel) const;

  // R and t are world-to-camera: point_cam = R * point_world + t.
  virtual bool Project(const Eigen::Matrix3d& R, const Eigen::Vector3d& t,
                       const Eigen::Vector3d& point_world,
                       Eigen::Vector2d* pixel) const;

  // Projects a point without checking whether the pixel lies inside the
  // image. Still returns false for points behind the camera.
  virtual bool ProjectWithoutRangeCheck(const Eigen::Vector3d& point_cam,
                                        Eigen::Vector2d* pixel) const;

  // Converts an image pixel into a unit ray in camera coordinates.
  Eigen::Vector3d PixelToRay(double x, double y) const;

  void Rescale(double scale);

  // Pixel coordinate -> undistorted normalized camera plane coordinate.
  virtual void ImageToNormalized(double x, double y, double* u,
                                 double* v) const = 0;

  // Undistorted normalized camera plane coordinate -> distorted pixel.
  virtual void NormalizedToImage(double u, double v, double* x,
                                 double* y) const = 0;

 protected:
  BaseCamera(CameraType type, size_t width, size_t height,
             std::vector<double> params);

  // Computes distortion offset. Distorted normalized coordinate is
  // (u + delta_u, v + delta_v).
  virtual void Distort(double u, double v, double* delta_u,
                       double* delta_v) const = 0;

  void IterativeUndistort(double distorted_u, double distorted_v, double* u,
                          double* v) const;

  CameraType type_ = CAMERA_TYPE_UNKNOWN;
  size_t width_ = 0;
  size_t height_ = 0;
  // pinhole: [fx, fy, cx, cy, k1, k2, p1, p2, k3]
  // fisheye: [fx, fy, cx, cy, k1, k2, k3, k4]
  std::vector<double> params_;
};

class OpenCVPinholeCamera : public BaseCamera {
 public:
  OpenCVPinholeCamera();
  OpenCVPinholeCamera(size_t width, size_t height, const Eigen::Matrix3d& K,
                      const std::vector<double>& distortion_coefficients);
  explicit OpenCVPinholeCamera(const CameraCalibration& calibration);

  std::string ModelName() const override;

  void ImageToNormalized(double x, double y, double* u,
                         double* v) const override;
  void NormalizedToImage(double u, double v, double* x,
                         double* y) const override;

 protected:
  void Distort(double u, double v, double* delta_u,
               double* delta_v) const override;
};

class OpenCVFisheyeCamera : public BaseCamera {
 public:
  OpenCVFisheyeCamera();
  OpenCVFisheyeCamera(size_t width, size_t height, const Eigen::Matrix3d& K,
                      const std::vector<double>& distortion_coefficients);
  explicit OpenCVFisheyeCamera(const CameraCalibration& calibration);

  std::string ModelName() const override;

  void ImageToNormalized(double x, double y, double* u,
                         double* v) const override;
  void NormalizedToImage(double u, double v, double* x,
                         double* y) const override;

 protected:
  void Distort(double u, double v, double* delta_u,
               double* delta_v) const override;
};

std::unique_ptr<BaseCamera> CreateCamera(const CameraCalibration& calibration);

}  // namespace mapping
}  // namespace adlabel
