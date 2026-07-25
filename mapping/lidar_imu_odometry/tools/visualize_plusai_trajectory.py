#!/usr/bin/env python3

import argparse
import csv
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

            pose = np.eye(4, dtype=np.float64)
            pose[:3, :4] = np.asarray(
                [float(value) for value in values], dtype=np.float64
            ).reshape(3, 4)
            poses.append(pose)

    if not poses:
        raise ValueError(f"trajectory contains no poses: {path}")
    return np.stack(poses)


def load_lidar_timestamps(path):
    timestamps = []
    with path.open("r", encoding="utf-8") as input_file:
        for line_number, line in enumerate(input_file, start=1):
            if not line.strip():
                continue
            try:
                timestamps.append(int(line))
            except ValueError as error:
                raise ValueError(
                    f"{path}:{line_number}: invalid nanosecond timestamp"
                ) from error

    if not timestamps:
        raise ValueError(f"timestamp file is empty: {path}")
    return np.asarray(timestamps, dtype=np.int64)


def normalize_quaternion(quaternion):
    norm = np.linalg.norm(quaternion)
    if norm <= 0.0:
        raise ValueError("encountered a zero-length quaternion")
    return quaternion / norm


def quaternion_to_rotation(quaternion):
    w, x, y, z = normalize_quaternion(quaternion)
    return np.asarray(
        [
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
            ],
            [
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
            ],
            [
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ],
        dtype=np.float64,
    )


def quaternion_slerp(first, second, ratio):
    first = normalize_quaternion(first)
    second = normalize_quaternion(second)
    dot = float(np.dot(first, second))
    if dot < 0.0:
        second = -second
        dot = -dot
    dot = float(np.clip(dot, -1.0, 1.0))

    if dot > 0.9995:
        return normalize_quaternion(first + ratio * (second - first))

    theta = math.acos(dot)
    sin_theta = math.sin(theta)
    first_weight = math.sin((1.0 - ratio) * theta) / sin_theta
    second_weight = math.sin(ratio * theta) / sin_theta
    return normalize_quaternion(
        first_weight * first + second_weight * second
    )


def load_gnss_samples(path):
    samples = {}
    required_fields = {
        "timestamp_msec",
        "position_x",
        "position_y",
        "position_z",
        "orientation_qx",
        "orientation_qy",
        "orientation_qz",
        "orientation_qw",
    }

    with path.open("r", encoding="utf-8", newline="") as input_file:
        reader = csv.DictReader(
            input_file, delimiter=" ", skipinitialspace=True
        )
        if reader.fieldnames is None:
            raise ValueError(f"GNSS file has no header: {path}")
        missing_fields = required_fields.difference(reader.fieldnames)
        if missing_fields:
            raise ValueError(
                f"{path}: missing fields: {sorted(missing_fields)}"
            )

        for row in reader:
            timestamp_ns = int(round(float(row["timestamp_msec"]) * 1e6))
            position = np.asarray(
                [
                    float(row["position_x"]),
                    float(row["position_y"]),
                    float(row["position_z"]),
                ],
                dtype=np.float64,
            )
            quaternion = normalize_quaternion(
                np.asarray(
                    [
                        float(row["orientation_qw"]),
                        float(row["orientation_qx"]),
                        float(row["orientation_qy"]),
                        float(row["orientation_qz"]),
                    ],
                    dtype=np.float64,
                )
            )
            # Keep the last sample when multiple records share one millisecond.
            samples[timestamp_ns] = (position, quaternion)

    if not samples:
        raise ValueError(f"GNSS file contains no samples: {path}")

    timestamps = np.asarray(sorted(samples), dtype=np.int64)
    positions = np.stack([samples[timestamp][0] for timestamp in timestamps])
    quaternions = np.stack(
        [samples[timestamp][1] for timestamp in timestamps]
    )
    return timestamps, positions, quaternions


def make_pose(position, quaternion):
    pose = np.eye(4, dtype=np.float64)
    pose[:3, :3] = quaternion_to_rotation(quaternion)
    pose[:3, 3] = position
    return pose


def interpolate_gnss_poses(
    lidar_timestamps,
    gnss_timestamps,
    gnss_positions,
    gnss_quaternions,
    max_time_gap_ns,
):
    matched_indices = []
    matched_poses = []
    for lidar_index, timestamp in enumerate(lidar_timestamps):
        after_index = int(np.searchsorted(gnss_timestamps, timestamp))
        if (
            after_index < len(gnss_timestamps)
            and gnss_timestamps[after_index] == timestamp
        ):
            matched_indices.append(lidar_index)
            matched_poses.append(
                make_pose(
                    gnss_positions[after_index],
                    gnss_quaternions[after_index],
                )
            )
            continue

        if after_index == 0 or after_index == len(gnss_timestamps):
            continue
        before_index = after_index - 1
        before_gap = timestamp - gnss_timestamps[before_index]
        after_gap = gnss_timestamps[after_index] - timestamp
        if before_gap > max_time_gap_ns or after_gap > max_time_gap_ns:
            continue

        interval = (
            gnss_timestamps[after_index] - gnss_timestamps[before_index]
        )
        ratio = float(timestamp - gnss_timestamps[before_index]) / float(
            interval
        )
        position = (
            (1.0 - ratio) * gnss_positions[before_index]
            + ratio * gnss_positions[after_index]
        )
        quaternion = quaternion_slerp(
            gnss_quaternions[before_index],
            gnss_quaternions[after_index],
            ratio,
        )
        matched_indices.append(lidar_index)
        matched_poses.append(make_pose(position, quaternion))

    if not matched_poses:
        raise ValueError("no LiDAR timestamps could be matched to GNSS")
    return np.asarray(matched_indices), np.stack(matched_poses)


def normalize_poses(poses):
    first_pose_inverse = np.linalg.inv(poses[0])
    return np.stack([first_pose_inverse @ pose for pose in poses])


def rotation_angle(rotation):
    cosine = np.clip((np.trace(rotation) - 1.0) * 0.5, -1.0, 1.0)
    return math.acos(cosine)


def calculate_errors(odometry_poses, ground_truth_poses, matched_indices):
    translation_errors = (
        odometry_poses[:, :3, 3] - ground_truth_poses[:, :3, 3]
    )
    ate_rmse = math.sqrt(
        np.mean(np.sum(translation_errors * translation_errors, axis=1))
    )

    translation_rpe = []
    rotation_rpe = []
    for index in range(1, len(odometry_poses)):
        if matched_indices[index] != matched_indices[index - 1] + 1:
            continue
        odometry_delta = (
            np.linalg.inv(odometry_poses[index - 1])
            @ odometry_poses[index]
        )
        ground_truth_delta = (
            np.linalg.inv(ground_truth_poses[index - 1])
            @ ground_truth_poses[index]
        )
        relative_error = np.linalg.inv(ground_truth_delta) @ odometry_delta
        translation_rpe.append(np.linalg.norm(relative_error[:3, 3]))
        rotation_rpe.append(rotation_angle(relative_error[:3, :3]))

    if not translation_rpe:
        raise ValueError("no consecutive matched poses available for RPE")
    translation_rpe_rmse = math.sqrt(np.mean(np.square(translation_rpe)))
    rotation_rpe_rmse_degrees = math.degrees(
        math.sqrt(np.mean(np.square(rotation_rpe)))
    )
    return ate_rmse, translation_rpe_rmse, rotation_rpe_rmse_degrees


def accumulated_distances(positions):
    deltas = positions[1:] - positions[:-1]
    step_distances = np.linalg.norm(deltas, axis=1)
    return np.concatenate(([0.0], np.cumsum(step_distances)))


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
    ate_rmse, translation_rpe, rotation_rpe = errors

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
        label="GNSS ground truth",
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
        label="GNSS end",
    )
    trajectory_axis.set_title(
        "PlusAI LiDAR odometry vs GNSS\n"
        f"{len(odometry_poses)} timestamp-aligned poses"
    )
    trajectory_axis.set_xlabel("First-pose x (m)")
    trajectory_axis.set_ylabel("First-pose y (m)")
    trajectory_axis.axis("equal")
    trajectory_axis.grid(True, linewidth=0.4, alpha=0.6)
    trajectory_axis.legend()
    trajectory_axis.text(
        0.02,
        0.98,
        f"ATE RMSE: {ate_rmse:.3f} m\n"
        f"RPE translation RMSE (1 frame): {translation_rpe:.4f} m\n"
        f"RPE rotation RMSE (1 frame): {rotation_rpe:.4f} deg",
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
        label="GNSS ground truth",
    )
    height_axis.set_title("Z profile")
    height_axis.set_xlabel("Accumulated distance (m)")
    height_axis.set_ylabel("First-pose z (m)")
    height_axis.grid(True, linewidth=0.4, alpha=0.6)
    height_axis.legend()
    figure.tight_layout()

    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_path, dpi=200)
    plt.close(figure)


def parse_arguments():
    parser = argparse.ArgumentParser(
        description="Visualize and evaluate PlusAI LiDAR odometry."
    )
    parser.add_argument(
        "--result_dir",
        type=Path,
        required=True,
        help=(
            "Directory containing trajectory.txt, timestamps.txt, and "
            "localization_gnss.txt."
        ),
    )
    parser.add_argument(
        "--max_time_gap_ms",
        type=float,
        default=100.0,
        help="Maximum interpolation gap on either side in milliseconds.",
    )
    return parser.parse_args()


def main():
    args = parse_arguments()
    result_dir = args.result_dir.expanduser().resolve()
    trajectory_path = result_dir / "trajectory.txt"
    timestamps_path = result_dir / "timestamps.txt"
    ground_truth_path = result_dir / "localization_gnss.txt"
    output_path = result_dir / "plusai_trajectory.png"

    if not result_dir.is_dir():
        raise NotADirectoryError(
            f"result directory does not exist: {result_dir}"
        )
    for path in (trajectory_path, timestamps_path, ground_truth_path):
        if not path.is_file():
            raise FileNotFoundError(f"input file does not exist: {path}")
    if args.max_time_gap_ms <= 0.0:
        raise ValueError("--max_time_gap_ms must be positive")

    odometry_poses = load_poses(trajectory_path)
    lidar_timestamps = load_lidar_timestamps(timestamps_path)
    if len(odometry_poses) != len(lidar_timestamps):
        raise ValueError(
            f"pose count {len(odometry_poses)} does not match timestamp "
            f"count {len(lidar_timestamps)}"
        )

    gnss_timestamps, gnss_positions, gnss_quaternions = load_gnss_samples(
        ground_truth_path
    )
    matched_indices, ground_truth_poses = interpolate_gnss_poses(
        lidar_timestamps,
        gnss_timestamps,
        gnss_positions,
        gnss_quaternions,
        int(round(args.max_time_gap_ms * 1e6)),
    )
    odometry_poses = odometry_poses[matched_indices]
    odometry_poses = normalize_poses(odometry_poses)
    ground_truth_poses = normalize_poses(ground_truth_poses)
    errors = calculate_errors(
        odometry_poses, ground_truth_poses, matched_indices
    )
    plot_trajectories(
        odometry_poses, ground_truth_poses, errors, output_path
    )

    ate_rmse, translation_rpe, rotation_rpe = errors
    print(
        f"Matched {len(matched_indices)}/{len(lidar_timestamps)} "
        "LiDAR poses to GNSS"
    )
    print(f"ATE RMSE: {ate_rmse:.6f} m")
    print(f"RPE translation RMSE (1 frame): {translation_rpe:.6f} m")
    print(f"RPE rotation RMSE (1 frame): {rotation_rpe:.6f} deg")
    print(f"Saved trajectory image: {output_path}")


if __name__ == "__main__":
    main()
