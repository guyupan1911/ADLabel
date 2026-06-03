#include "mapping/camera/camera.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>

#include <Eigen/Dense>
#include <Eigen/LU>
#include <glog/logging.h>

namespace adlabel {
namespace mapping {
namespace {

constexpr size_t kBaseParamCount = 4;
constexpr size_t kOpenCVPinholeDistortionCount = 5;
constexpr size_t kOpenCVFisheyeDistortionCount = 4;

Eigen::Matrix3d CalibrationMatrixFromProto(const CameraCalibration& calibration) {
    CHECK(calibration.intrinsics_size() == 4 || calibration.intrinsics_size() == 9)
            << "camera intrinsics must be [fx, fy, cx, cy] or a row-major 3x3 K";

    Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
    if (calibration.intrinsics_size() == 4) {
        K(0, 0) = calibration.intrinsics(0);
        K(1, 1) = calibration.intrinsics(1);
        K(0, 2) = calibration.intrinsics(2);
        K(1, 2) = calibration.intrinsics(3);
        return K;
    }

    K(0, 0) = calibration.intrinsics(0);
    K(0, 1) = calibration.intrinsics(1);
    K(0, 2) = calibration.intrinsics(2);
    K(1, 0) = calibration.intrinsics(3);
    K(1, 1) = calibration.intrinsics(4);
    K(1, 2) = calibration.intrinsics(5);
    K(2, 0) = calibration.intrinsics(6);
    K(2, 1) = calibration.intrinsics(7);
    K(2, 2) = calibration.intrinsics(8);
    return K;
}

size_t WidthFromProto(const CameraCalibration& calibration) {
    CHECK(calibration.has_width()) << "camera calibration missing width";
    CHECK_GT(calibration.width(), 0u) << "camera width must be positive";
    return static_cast<size_t>(calibration.width());
}

size_t HeightFromProto(const CameraCalibration& calibration) {
    CHECK(calibration.has_height()) << "camera calibration missing height";
    CHECK_GT(calibration.height(), 0u) << "camera height must be positive";
    return static_cast<size_t>(calibration.height());
}

std::vector<double> BaseParamsFromK(const Eigen::Matrix3d& K) {
    CHECK_GT(K(0, 0), 0.0) << "fx must be positive";
    CHECK_GT(K(1, 1), 0.0) << "fy must be positive";
    return {K(0, 0), K(1, 1), K(0, 2), K(1, 2)};
}

std::vector<double> DistortionFromProto(const CameraCalibration& calibration) {
    std::vector<double> distortion;
    distortion.reserve(static_cast<size_t>(calibration.distortion_coefficients_size()));
    for (int i = 0; i < calibration.distortion_coefficients_size(); ++i) {
        distortion.push_back(calibration.distortion_coefficients(i));
    }
    return distortion;
}

std::vector<double> OpenCVPinholeParams(const Eigen::Matrix3d& K,
                                        const std::vector<double>& distortion) {
    CHECK(distortion.size() == 4 || distortion.size() == 5)
            << "OpenCV pinhole distortion must be [k1, k2, p1, p2] "
            << "or [k1, k2, p1, p2, k3]";

    std::vector<double> params = BaseParamsFromK(K);
    params.insert(params.end(), distortion.begin(), distortion.end());
    if (distortion.size() == 4) {
        params.push_back(0.0);
    }
    CHECK_EQ(params.size(), kBaseParamCount + kOpenCVPinholeDistortionCount);
    return params;
}

std::vector<double> OpenCVFisheyeParams(const Eigen::Matrix3d& K,
                                        const std::vector<double>& distortion) {
    CHECK_EQ(distortion.size(), kOpenCVFisheyeDistortionCount)
            << "OpenCV fisheye distortion must be [k1, k2, k3, k4]";

    std::vector<double> params = BaseParamsFromK(K);
    params.insert(params.end(), distortion.begin(), distortion.end());
    CHECK_EQ(params.size(), kBaseParamCount + kOpenCVFisheyeDistortionCount);
    return params;
}

void CheckCalibrationType(const CameraCalibration& calibration, CameraType expected_type) {
    CHECK(calibration.has_camera_type()) << "camera calibration missing camera_type";
    CHECK_EQ(calibration.camera_type(), expected_type)
            << "camera calibration type does not match the requested camera model";
}

}  // namespace

BaseCamera::BaseCamera(CameraType type,
                       size_t width,
                       size_t height,
                       std::vector<double> params)
    : type_(type), width_(width), height_(height), params_(std::move(params)) {
    CHECK_GT(width_, 0u) << "camera width must be positive";
    CHECK_GT(height_, 0u) << "camera height must be positive";
    CHECK_GE(params_.size(), kBaseParamCount);
    CHECK_GT(FocalLengthX(), 0.0) << "fx must be positive";
    CHECK_GT(FocalLengthY(), 0.0) << "fy must be positive";
}

std::vector<double> BaseCamera::DistortionCoefficients() const {
    if (params_.size() <= kBaseParamCount) {
        return {};
    }
    return std::vector<double>(params_.begin() + kBaseParamCount, params_.end());
}

Eigen::Matrix3d BaseCamera::CalibrationMatrix() const {
    Eigen::Matrix3d K = Eigen::Matrix3d::Identity();
    K(0, 0) = FocalLengthX();
    K(1, 1) = FocalLengthY();
    K(0, 2) = PrincipalPointX();
    K(1, 2) = PrincipalPointY();
    return K;
}

CameraCalibration BaseCamera::ToProto() const {
    CameraCalibration calibration;
    calibration.set_camera_type(type_);
    calibration.set_width(static_cast<uint32_t>(width_));
    calibration.set_height(static_cast<uint32_t>(height_));
    calibration.add_intrinsics(FocalLengthX());
    calibration.add_intrinsics(FocalLengthY());
    calibration.add_intrinsics(PrincipalPointX());
    calibration.add_intrinsics(PrincipalPointY());
    for (double value : DistortionCoefficients()) {
        calibration.add_distortion_coefficients(value);
    }
    return calibration;
}

bool BaseCamera::Project(const Eigen::Vector3d& point_cam, Eigen::Vector2d* pixel) const {
    CHECK(pixel != nullptr);
    if (!ProjectWithoutRangeCheck(point_cam, pixel)) {
        return false;
    }
    return pixel->x() >= 0.0 && pixel->x() < static_cast<double>(width_) &&
           pixel->y() >= 0.0 && pixel->y() < static_cast<double>(height_);
}

bool BaseCamera::Project(const Eigen::Matrix3d& R,
                         const Eigen::Vector3d& t,
                         const Eigen::Vector3d& point_world,
                         Eigen::Vector2d* pixel) const {
    return Project(R * point_world + t, pixel);
}

bool BaseCamera::ProjectWithoutRangeCheck(const Eigen::Vector3d& point_cam,
                                          Eigen::Vector2d* pixel) const {
    CHECK(pixel != nullptr);
    if (point_cam.z() <= 0.0) {
        return false;
    }
    const double u = point_cam.x() / point_cam.z();
    const double v = point_cam.y() / point_cam.z();
    NormalizedToImage(u, v, &(*pixel)(0), &(*pixel)(1));
    return true;
}

Eigen::Vector3d BaseCamera::PixelToRay(double x, double y) const {
    double u = 0.0;
    double v = 0.0;
    ImageToNormalized(x, y, &u, &v);
    return Eigen::Vector3d(u, v, 1.0).normalized();
}

void BaseCamera::Rescale(double scale) {
    CHECK_GT(scale, 0.0) << "scale must be positive";
    CHECK_GT(width_, 0u);
    CHECK_GT(height_, 0u);

    const size_t new_width = static_cast<size_t>(std::lround(scale * width_));
    const size_t new_height = static_cast<size_t>(std::lround(scale * height_));
    CHECK_GT(new_width, 0u);
    CHECK_GT(new_height, 0u);

    const double scale_x = static_cast<double>(new_width) / static_cast<double>(width_);
    const double scale_y = static_cast<double>(new_height) / static_cast<double>(height_);
    width_ = new_width;
    height_ = new_height;
    params_[0] *= scale_x;
    params_[1] *= scale_y;
    params_[2] *= scale_x;
    params_[3] *= scale_y;
}

void BaseCamera::IterativeUndistort(double distorted_u,
                                    double distorted_v,
                                    double* u,
                                    double* v) const {
    CHECK(u != nullptr);
    CHECK(v != nullptr);

    constexpr size_t kMaxIterations = 100;
    constexpr double kMaxStepSquaredNorm = 1e-10;
    constexpr double kRelStepSize = 1e-6;

    const Eigen::Vector2d x0(distorted_u, distorted_v);
    Eigen::Vector2d x(distorted_u, distorted_v);

    for (size_t i = 0; i < kMaxIterations; ++i) {
        const double step0 = std::max(std::numeric_limits<double>::epsilon(),
                                      std::abs(kRelStepSize * x(0)));
        const double step1 = std::max(std::numeric_limits<double>::epsilon(),
                                      std::abs(kRelStepSize * x(1)));

        Eigen::Vector2d dx;
        Eigen::Vector2d dx_0b;
        Eigen::Vector2d dx_0f;
        Eigen::Vector2d dx_1b;
        Eigen::Vector2d dx_1f;
        Distort(x(0), x(1), &dx(0), &dx(1));
        Distort(x(0) - step0, x(1), &dx_0b(0), &dx_0b(1));
        Distort(x(0) + step0, x(1), &dx_0f(0), &dx_0f(1));
        Distort(x(0), x(1) - step1, &dx_1b(0), &dx_1b(1));
        Distort(x(0), x(1) + step1, &dx_1f(0), &dx_1f(1));

        Eigen::Matrix2d J;
        J(0, 0) = 1.0 + (dx_0f(0) - dx_0b(0)) / (2.0 * step0);
        J(0, 1) = (dx_1f(0) - dx_1b(0)) / (2.0 * step1);
        J(1, 0) = (dx_0f(1) - dx_0b(1)) / (2.0 * step0);
        J(1, 1) = 1.0 + (dx_1f(1) - dx_1b(1)) / (2.0 * step1);

        const Eigen::FullPivLU<Eigen::Matrix2d> lu(J);
        if (!lu.isInvertible()) {
            LOG(WARNING) << "failed to invert undistortion Jacobian";
            break;
        }

        const Eigen::Vector2d step_x = lu.solve(x + dx - x0);
        x -= step_x;
        if (step_x.squaredNorm() < kMaxStepSquaredNorm) {
            break;
        }
    }

    *u = x(0);
    *v = x(1);
}

OpenCVPinholeCamera::OpenCVPinholeCamera()
    : BaseCamera(CAMERA_TYPE_PINHOLE,
                 1,
                 1,
                 {1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}) {}

OpenCVPinholeCamera::OpenCVPinholeCamera(
        size_t width,
        size_t height,
        const Eigen::Matrix3d& K,
        const std::vector<double>& distortion_coefficients)
    : BaseCamera(CAMERA_TYPE_PINHOLE,
                 width,
                 height,
                 OpenCVPinholeParams(K, distortion_coefficients)) {}

OpenCVPinholeCamera::OpenCVPinholeCamera(const CameraCalibration& calibration)
    : OpenCVPinholeCamera(WidthFromProto(calibration),
                          HeightFromProto(calibration),
                          CalibrationMatrixFromProto(calibration),
                          DistortionFromProto(calibration)) {
    CheckCalibrationType(calibration, CAMERA_TYPE_PINHOLE);
}

std::string OpenCVPinholeCamera::ModelName() const {
    return "OpenCVPinholeCamera";
}

void OpenCVPinholeCamera::ImageToNormalized(double x,
                                            double y,
                                            double* u,
                                            double* v) const {
    CHECK(u != nullptr);
    CHECK(v != nullptr);
    const double distorted_u = (x - PrincipalPointX()) / FocalLengthX();
    const double distorted_v = (y - PrincipalPointY()) / FocalLengthY();
    IterativeUndistort(distorted_u, distorted_v, u, v);
}

void OpenCVPinholeCamera::NormalizedToImage(double u,
                                            double v,
                                            double* x,
                                            double* y) const {
    CHECK(x != nullptr);
    CHECK(y != nullptr);
    double du = 0.0;
    double dv = 0.0;
    Distort(u, v, &du, &dv);
    *x = FocalLengthX() * (u + du) + PrincipalPointX();
    *y = FocalLengthY() * (v + dv) + PrincipalPointY();
}

void OpenCVPinholeCamera::Distort(double u,
                                  double v,
                                  double* delta_u,
                                  double* delta_v) const {
    CHECK(delta_u != nullptr);
    CHECK(delta_v != nullptr);
    const double k1 = params_[4];
    const double k2 = params_[5];
    const double p1 = params_[6];
    const double p2 = params_[7];
    const double k3 = params_[8];

    const double u2 = u * u;
    const double v2 = v * v;
    const double uv = u * v;
    const double r2 = u2 + v2;
    const double r4 = r2 * r2;
    const double r6 = r2 * r4;
    const double radial = 1.0 + k1 * r2 + k2 * r4 + k3 * r6;

    *delta_u = u * radial + 2.0 * p1 * uv + p2 * (r2 + 2.0 * u2) - u;
    *delta_v = v * radial + p1 * (r2 + 2.0 * v2) + 2.0 * p2 * uv - v;
}

OpenCVFisheyeCamera::OpenCVFisheyeCamera()
    : BaseCamera(CAMERA_TYPE_FISHEYE,
                 1,
                 1,
                 {1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}) {}

OpenCVFisheyeCamera::OpenCVFisheyeCamera(
        size_t width,
        size_t height,
        const Eigen::Matrix3d& K,
        const std::vector<double>& distortion_coefficients)
    : BaseCamera(CAMERA_TYPE_FISHEYE,
                 width,
                 height,
                 OpenCVFisheyeParams(K, distortion_coefficients)) {}

OpenCVFisheyeCamera::OpenCVFisheyeCamera(const CameraCalibration& calibration)
    : OpenCVFisheyeCamera(WidthFromProto(calibration),
                          HeightFromProto(calibration),
                          CalibrationMatrixFromProto(calibration),
                          DistortionFromProto(calibration)) {
    CheckCalibrationType(calibration, CAMERA_TYPE_FISHEYE);
}

std::string OpenCVFisheyeCamera::ModelName() const {
    return "OpenCVFisheyeCamera";
}

void OpenCVFisheyeCamera::ImageToNormalized(double x,
                                            double y,
                                            double* u,
                                            double* v) const {
    CHECK(u != nullptr);
    CHECK(v != nullptr);
    const double distorted_u = (x - PrincipalPointX()) / FocalLengthX();
    const double distorted_v = (y - PrincipalPointY()) / FocalLengthY();
    IterativeUndistort(distorted_u, distorted_v, u, v);
}

void OpenCVFisheyeCamera::NormalizedToImage(double u,
                                            double v,
                                            double* x,
                                            double* y) const {
    CHECK(x != nullptr);
    CHECK(y != nullptr);
    double du = 0.0;
    double dv = 0.0;
    Distort(u, v, &du, &dv);
    *x = FocalLengthX() * (u + du) + PrincipalPointX();
    *y = FocalLengthY() * (v + dv) + PrincipalPointY();
}

void OpenCVFisheyeCamera::Distort(double u,
                                  double v,
                                  double* delta_u,
                                  double* delta_v) const {
    CHECK(delta_u != nullptr);
    CHECK(delta_v != nullptr);
    const double k1 = params_[4];
    const double k2 = params_[5];
    const double k3 = params_[6];
    const double k4 = params_[7];

    const double r = std::sqrt(u * u + v * v);
    constexpr double kThreshold = 1e-12;
    if (r <= kThreshold) {
        *delta_u = 0.0;
        *delta_v = 0.0;
        return;
    }

    const double theta = std::atan(r);
    const double theta2 = theta * theta;
    const double theta4 = theta2 * theta2;
    const double theta6 = theta4 * theta2;
    const double theta8 = theta4 * theta4;
    const double theta_d =
            theta * (1.0 + k1 * theta2 + k2 * theta4 + k3 * theta6 + k4 * theta8);
    const double ratio = theta_d / r;
    *delta_u = u * ratio - u;
    *delta_v = v * ratio - v;
}

std::unique_ptr<BaseCamera> CreateCamera(const CameraCalibration& calibration) {
    CHECK(calibration.has_camera_type()) << "camera calibration missing camera_type";
    switch (calibration.camera_type()) {
        case CAMERA_TYPE_PINHOLE:
            return std::make_unique<OpenCVPinholeCamera>(calibration);
        case CAMERA_TYPE_FISHEYE:
            return std::make_unique<OpenCVFisheyeCamera>(calibration);
        default:
            LOG(FATAL) << "unsupported camera type: " << calibration.camera_type();
    }
    return nullptr;
}

}  // namespace mapping
}  // namespace adlabel
