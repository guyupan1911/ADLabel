#!/usr/bin/env python3

import argparse
import math
from pathlib import Path


def default_trajectory_path():
    repo_root = Path(__file__).resolve().parents[3]
    return repo_root / "data" / "kitti" / "kitti_odom_result"


def load_trajectory(path):
    positions = []
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
                pose = [float(value) for value in values]
            except ValueError as error:
                raise ValueError(
                    f"{path}:{line_number}: contains a non-numeric value"
                ) from error

            positions.append((pose[3], pose[7], pose[11]))

    if not positions:
        raise ValueError(f"trajectory contains no poses: {path}")
    return positions


def accumulated_distances(positions):
    distances = [0.0]
    for previous, current in zip(positions, positions[1:]):
        distances.append(distances[-1] + math.dist(previous, current))
    return distances


def plot_trajectory(positions, output_path):
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    xs = [position[0] for position in positions]
    ys = [position[1] for position in positions]
    zs = [position[2] for position in positions]
    distances = accumulated_distances(positions)
    length = distances[-1]

    figure, (trajectory_axis, height_axis) = plt.subplots(
        2, 1, figsize=(12, 14), gridspec_kw={"height_ratios": [3, 1]}
    )
    trajectory_axis.plot(
        xs, ys, color="#1f77b4", linewidth=1.2, label="trajectory"
    )
    trajectory_axis.scatter(
        xs[0], ys[0], color="#2ca02c", marker="o", s=60, label="start"
    )
    trajectory_axis.scatter(
        xs[-1], ys[-1], color="#d62728", marker="x", s=70, label="end"
    )

    trajectory_axis.set_title(
        f"KITTI LiDAR odometry trajectory\n"
        f"{len(positions)} poses, length {length:.1f} m"
    )
    trajectory_axis.set_xlabel("LiDAR x (m)")
    trajectory_axis.set_ylabel("LiDAR y (m)")
    trajectory_axis.axis("equal")
    trajectory_axis.grid(True, linewidth=0.4, alpha=0.6)
    trajectory_axis.legend()

    height_axis.plot(distances, zs, color="#ff7f0e", linewidth=1.0)
    height_axis.set_title("LiDAR z profile")
    height_axis.set_xlabel("Accumulated distance (m)")
    height_axis.set_ylabel("LiDAR z (m)")
    height_axis.grid(True, linewidth=0.4, alpha=0.6)
    figure.tight_layout()

    output_path.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(output_path, dpi=200)
    print(f"Loaded {len(positions)} poses")
    print(f"Trajectory length: {length:.3f} m")
    print(f"Saved trajectory image: {output_path}")

    plt.close(figure)


def parse_arguments():
    default_input = default_trajectory_path()
    parser = argparse.ArgumentParser(
        description="Visualize an ADLabel KITTI-format LiDAR trajectory."
    )
    parser.add_argument(
        "--trajectory_path",
        type=Path,
        default=default_input,
        help=f"KITTI-format trajectory file (default: {default_input})",
    )
    parser.add_argument(
        "--output_path",
        type=Path,
        default=None,
        help="Output PNG path (default: <trajectory_path>.png)",
    )
    return parser.parse_args()


def main():
    args = parse_arguments()
    trajectory_path = args.trajectory_path.expanduser().resolve()
    output_path = args.output_path
    if output_path is None:
        output_path = Path(f"{trajectory_path}.png")
    else:
        output_path = output_path.expanduser().resolve()

    if not trajectory_path.is_file():
        raise FileNotFoundError(f"trajectory file does not exist: {trajectory_path}")

    positions = load_trajectory(trajectory_path)
    plot_trajectory(positions, output_path)


if __name__ == "__main__":
    main()
