#!/usr/bin/env python3

import argparse
import math
from pathlib import Path

import numpy as np


def load_poses(path):
    poses = []
    with path.open("r", encoding="utf-8") as input_file:
        for line_number, line in enumerate(input_file, start=1):
            if not line.strip():
                continue

            values = line.split()
            if len(values) != 12:
                raise ValueError(
                    f"{path}:{line_number}: expected 12 values, "
                    f"but got {len(values)}"
                )
            try:
                matrix_values = np.array(
                    [float(value) for value in values], dtype=np.float64
                )
            except ValueError as error:
                raise ValueError(
                    f"{path}:{line_number}: contains a non-numeric value"
                ) from error

            pose = np.eye(4, dtype=np.float64)
            pose[:3, :4] = matrix_values.reshape(3, 4)
            poses.append(pose)

    if not poses:
        raise ValueError(f"trajectory contains no poses: {path}")
    return np.stack(poses)


def load_lidar_to_camera(calibration_path):
    with calibration_path.open("r", encoding="utf-8") as input_file:
        for line in input_file:
            label, separator, values = line.partition(":")
            if separator and label.strip() == "Tr":
                matrix_values = np.array(
                    [float(value) for value in values.split()],
                    dtype=np.float64,
                )
                if matrix_values.size != 12:
                    raise ValueError(
                        f"{calibration_path}: Tr must contain 12 values"
                    )
                transform = np.eye(4, dtype=np.float64)
                transform[:3, :4] = matrix_values.reshape(3, 4)
                return transform

    raise ValueError(f"{calibration_path}: missing Tr calibration")


def normalize_poses(poses):
    first_pose_inverse = np.linalg.inv(poses[0])
    return np.stack([first_pose_inverse @ pose for pose in poses])


def camera_to_lidar_poses(camera_poses, lidar_to_camera):
    camera_to_lidar = np.linalg.inv(lidar_to_camera)
    return np.stack(
        [
            camera_to_lidar @ pose @ lidar_to_camera
            for pose in camera_poses
        ]
    )


def accumulated_distances(positions):
    distances = [0.0]
    for previous, current in zip(positions, positions[1:]):
        distances.append(distances[-1] + math.dist(previous, current))
    return distances


def rotation_angle(rotation):
    cosine = np.clip((np.trace(rotation) - 1.0) * 0.5, -1.0, 1.0)
    return math.acos(cosine)


def calculate_errors(odometry_poses, ground_truth_poses):
    translation_errors = (
        odometry_poses[:, :3, 3] - ground_truth_poses[:, :3, 3]
    )
    ate_rmse = math.sqrt(
        np.mean(np.sum(translation_errors * translation_errors, axis=1))
    )

    rpe_translation_errors = []
    rpe_rotation_errors = []
    for index in range(1, len(odometry_poses)):
        odometry_delta = (
            np.linalg.inv(odometry_poses[index - 1])
            @ odometry_poses[index]
        )
        ground_truth_delta = (
            np.linalg.inv(ground_truth_poses[index - 1])
            @ ground_truth_poses[index]
        )
        relative_error = np.linalg.inv(ground_truth_delta) @ odometry_delta
        rpe_translation_errors.append(
            np.linalg.norm(relative_error[:3, 3])
        )
        rpe_rotation_errors.append(rotation_angle(relative_error[:3, :3]))

    rpe_translation_rmse = math.sqrt(
        np.mean(np.square(rpe_translation_errors))
    )
    rpe_rotation_rmse_degrees = math.degrees(
        math.sqrt(np.mean(np.square(rpe_rotation_errors)))
    )
    return ate_rmse, rpe_translation_rmse, rpe_rotation_rmse_degrees


def plot_trajectories(
    odometry_poses, ground_truth_poses, errors, output_path
):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    odometry_positions = odometry_poses[:, :3, 3]
    ground_truth_positions = ground_truth_poses[:, :3, 3]
    odometry_distances = accumulated_distances(odometry_positions)
    ground_truth_distances = accumulated_distances(ground_truth_positions)
    odometry_length = odometry_distances[-1]
    ate_rmse, rpe_translation_rmse, rpe_rotation_rmse_degrees = errors

    figure, (trajectory_axis, height_axis) = plt.subplots(
        2, 1, figsize=(12, 14), gridspec_kw={"height_ratios": [3, 1]}
    )
    trajectory_axis.plot(
        odometry_positions[:, 0],
        odometry_positions[:, 1],
        color="#1f77b4",
        linewidth=1.2,
        label="LiDAR odometry",
    )
    trajectory_axis.plot(
        ground_truth_positions[:, 0],
        ground_truth_positions[:, 1],
        color="#2ca02c",
        linewidth=1.2,
        label="Ground truth",
    )
    trajectory_axis.scatter(
        odometry_positions[0, 0],
        odometry_positions[0, 1],
        color="black",
        marker="o",
        s=60,
        label="Start",
    )
    trajectory_axis.scatter(
        odometry_positions[-1, 0],
        odometry_positions[-1, 1],
        color="#1f77b4",
        marker="x",
        s=70,
        label="Odometry end",
    )
    trajectory_axis.scatter(
        ground_truth_positions[-1, 0],
        ground_truth_positions[-1, 1],
        color="#2ca02c",
        marker="x",
        s=70,
        label="Ground truth end",
    )

    trajectory_axis.set_title(
        f"KITTI LiDAR odometry trajectory\n"
        f"{len(odometry_poses)} poses, odometry length "
        f"{odometry_length:.1f} m"
    )
    trajectory_axis.set_xlabel("LiDAR x (m)")
    trajectory_axis.set_ylabel("LiDAR y (m)")
    trajectory_axis.axis("equal")
    trajectory_axis.grid(True, linewidth=0.4, alpha=0.6)
    trajectory_axis.legend()
    trajectory_axis.text(
        0.02,
        0.98,
        f"ATE RMSE: {ate_rmse:.3f} m\n"
        f"RPE translation RMSE (1 frame): "
        f"{rpe_translation_rmse:.4f} m\n"
        f"RPE rotation RMSE (1 frame): "
        f"{rpe_rotation_rmse_degrees:.4f} deg",
        transform=trajectory_axis.transAxes,
        verticalalignment="top",
        bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.85},
    )

    height_axis.plot(
        odometry_distances,
        odometry_positions[:, 2],
        color="#1f77b4",
        linewidth=1.0,
        label="LiDAR odometry",
    )
    height_axis.plot(
        ground_truth_distances,
        ground_truth_positions[:, 2],
        color="#2ca02c",
        linewidth=1.0,
        label="Ground truth",
    )
    height_axis.set_title("LiDAR z profile")
    height_axis.set_xlabel("Accumulated distance (m)")
    height_axis.set_ylabel("LiDAR z (m)")
    height_axis.grid(True, linewidth=0.4, alpha=0.6)
    height_axis.legend()
    figure.tight_layout()

    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_path, dpi=200)
    plt.close(figure)


def parse_arguments():
    default_ground_truth_path = Path("data/kitti/00.txt")
    parser = argparse.ArgumentParser(
        description="Visualize and evaluate a KITTI LiDAR trajectory."
    )
    parser.add_argument(
        "--trajectory_path",
        type=Path,
        required=True,
        help="KITTI-format LiDAR odometry trajectory file.",
    )
    parser.add_argument(
        "--ground_truth_path",
        type=Path,
        default=default_ground_truth_path,
        help=(
            "KITTI camera ground-truth file "
            f"(default: {default_ground_truth_path})"
        ),
    )
    parser.add_argument(
        "--output_dir",
        type=Path,
        required=True,
        help="Directory for the trajectory PNG.",
    )
    return parser.parse_args()


def main():
    args = parse_arguments()
    trajectory_path = args.trajectory_path.expanduser().resolve()
    ground_truth_path = args.ground_truth_path.expanduser().resolve()
    output_path = args.output_dir.expanduser().resolve() / "trajectory.png"

    if not trajectory_path.is_file():
        raise FileNotFoundError(
            f"trajectory file does not exist: {trajectory_path}"
        )
    if not ground_truth_path.is_file():
        raise FileNotFoundError(
            f"ground-truth file does not exist: {ground_truth_path}"
        )

    calibration_path = (
        ground_truth_path.parent / ground_truth_path.stem / "calib.txt"
    )
    if not calibration_path.is_file():
        raise FileNotFoundError(
            f"calibration file does not exist: {calibration_path}"
        )

    odometry_poses = normalize_poses(load_poses(trajectory_path))
    ground_truth_camera_poses = load_poses(ground_truth_path)
    lidar_to_camera = load_lidar_to_camera(calibration_path)
    ground_truth_lidar_poses = normalize_poses(
        camera_to_lidar_poses(ground_truth_camera_poses, lidar_to_camera)
    )

    if len(odometry_poses) > len(ground_truth_lidar_poses):
        raise ValueError(
            f"odometry has {len(odometry_poses)} poses, but ground truth "
            f"has only {len(ground_truth_lidar_poses)}"
        )
    ground_truth_lidar_poses = ground_truth_lidar_poses[
        : len(odometry_poses)
    ]
    if len(odometry_poses) < 2:
        raise ValueError("at least two poses are required to calculate RPE")

    errors = calculate_errors(odometry_poses, ground_truth_lidar_poses)
    plot_trajectories(
        odometry_poses, ground_truth_lidar_poses, errors, output_path
    )

    ate_rmse, rpe_translation_rmse, rpe_rotation_rmse_degrees = errors
    print(f"Loaded {len(odometry_poses)} odometry poses")
    print(f"ATE RMSE: {ate_rmse:.6f} m")
    print(
        "RPE translation RMSE (1 frame): "
        f"{rpe_translation_rmse:.6f} m"
    )
    print(
        "RPE rotation RMSE (1 frame): "
        f"{rpe_rotation_rmse_degrees:.6f} deg"
    )
    print(f"Saved trajectory image: {output_path}")


if __name__ == "__main__":
    main()
