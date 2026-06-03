# ADLabel Camera Model Design

Module path: `mapping/camera`

## Scope

This module stores camera intrinsics and distortion parameters only. It does not
store camera extrinsics. Callers are responsible for transforming lidar, imu, or
world points into the camera coordinate system before calling `Project()`.

Supported models:

```text
BaseCamera
├── OpenCVPinholeCamera
└── OpenCVFisheyeCamera
```

Intentionally not migrated from the source project:

```text
RadialCamera
OpenCVExtendCamera
FOVCamera
PanoramaCamera
Grid<float> undistortion maps
Ideal no-distortion pinhole model
BA parameter-group APIs
ProjectLine
```

## Coordinate Chain

```text
camera point (Xc, Yc, Zc), z forward
        |
        | Project()
        v
undistorted normalized plane (u, v) = (Xc / Zc, Yc / Zc)
        |
        | NormalizedToImage()
        v
distorted pixel (x, y)
```

`Project()` returns false when `Zc <= 0` or the projected pixel is outside the
image. `ProjectWithoutRangeCheck()` still rejects `Zc <= 0`, but keeps pixels
outside the image.

## Public Interfaces

Important interfaces in `BaseCamera`:

```cpp
CameraType Type() const;
std::string ModelName() const;

size_t Width() const;
size_t Height() const;

double FocalLengthX() const;
double FocalLengthY() const;
double PrincipalPointX() const;
double PrincipalPointY() const;

Eigen::Matrix3d CalibrationMatrix() const;
CameraCalibration ToProto() const;

bool Project(const Eigen::Vector3d& point_cam, Eigen::Vector2d* pixel) const;
bool Project(const Eigen::Matrix3d& R,
             const Eigen::Vector3d& t,
             const Eigen::Vector3d& point_world,
             Eigen::Vector2d* pixel) const;
bool ProjectWithoutRangeCheck(const Eigen::Vector3d& point_cam,
                              Eigen::Vector2d* pixel) const;

Eigen::Vector3d PixelToRay(double x, double y) const;
void Rescale(double scale);

void ImageToNormalized(double x, double y, double* u, double* v) const;
void NormalizedToImage(double u, double v, double* x, double* y) const;
```

The `R, t` overload uses world-to-camera convention:

```text
point_cam = R * point_world + t
```

## Parameter Layout

All models store parameters in one vector:

```text
OpenCVPinholeCamera:
  [fx, fy, cx, cy, k1, k2, p1, p2, k3]

OpenCVFisheyeCamera:
  [fx, fy, cx, cy, k1, k2, k3, k4]
```

The proto accepts intrinsics as either:

```text
[fx, fy, cx, cy]
```

or a row-major 3x3 matrix:

```text
[fx, skew, cx,
  0,  fy,   cy,
  0,  0,    1]
```

The implementation ignores skew in projection and uses `fx`, `fy`, `cx`, `cy`.

## Distortion Models

### OpenCVPinholeCamera

This is the standard OpenCV pinhole distortion model with radial and tangential
terms:

```text
r2 = u^2 + v^2
radial = 1 + k1*r2 + k2*r2^2 + k3*r2^3

u_d = u * radial + 2*p1*u*v + p2*(r2 + 2*u^2)
v_d = v * radial + p1*(r2 + 2*v^2) + 2*p2*u*v

x = fx * u_d + cx
y = fy * v_d + cy
```

The proto may provide `[k1, k2, p1, p2]`; in that case `k3` is set to zero.

### OpenCVFisheyeCamera

This is the OpenCV fisheye / Kannala-Brandt model:

```text
r = sqrt(u^2 + v^2)
theta = atan(r)
theta_d = theta * (1 + k1*theta^2 + k2*theta^4 + k3*theta^6 + k4*theta^8)

u_d = (theta_d / r) * u
v_d = (theta_d / r) * v

x = fx * u_d + cx
y = fy * v_d + cy
```

## Undistortion

`ImageToNormalized()` removes the camera matrix first, then calls
`IterativeUndistort()` for distorted models:

```text
pixel -> distorted normalized coordinate -> undistorted normalized coordinate
```

The iterative solver is inherited from the source project. It numerically
estimates the 2x2 Jacobian of the distortion function and solves a Newton step.

## Proto Mapping

`mapping/protos/camera_calibration.proto` defines:

```proto
CAMERA_TYPE_PINHOLE
CAMERA_TYPE_FISHEYE
```

`CAMERA_TYPE_PINHOLE` maps to `OpenCVPinholeCamera`, not to an ideal
no-distortion pinhole camera. `CAMERA_TYPE_FISHEYE` maps to
`OpenCVFisheyeCamera`.

Use the factory to build a camera from a frame:

```cpp
std::unique_ptr<BaseCamera> camera = CreateCamera(frame.camera_calibration());
```
