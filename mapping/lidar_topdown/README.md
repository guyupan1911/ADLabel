# Lidar Topdown

This module rasterizes lidar points into top-down BEV grid images.

Main class:

- `LidarLosslessMapNode`: grid accumulator for intensity and altitude statistics.

Coordinate convention for ADLabel:

```text
lidar point
  -> Frame::sensor_to_imu_extrinsic
  -> current IMU frame
  -> Frame::lio_pose_3d
  -> world frame
  -> GridFrame::WorldToPixel
```

The frame loading, dynamic-point filtering, and pose transforms are handled by
the caller. This module only accumulates already transformed points into a BEV
grid.

The output grid uses image coordinates:

```text
col increases with +x
row increases with -y
```

Empty intensity cells are written as `0`. Empty altitude cells are written as
`NaN` for `CV_32FC1` altitude images and `0` for rescaled `CV_16UC1` altitude
images.
