#!/usr/bin/env python3

import argparse
import csv
import math
import os
from collections import defaultdict

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse


RESULTS_DIR = "data/gtsam_results"
DEFAULT_CSV = os.path.join(RESULTS_DIR, "odometry_trajectory.csv")
DEFAULT_OUT = os.path.join(RESULTS_DIR, "odometry_trajectory.png")


def parse_args():
    parser = argparse.ArgumentParser(description="Plot GTSAM Pose2 trajectories from CSV.")
    parser.add_argument("--csv", default=DEFAULT_CSV, help="Input trajectory CSV.")
    parser.add_argument("--out", default=DEFAULT_OUT, help="Output image path, usually .png.")
    parser.add_argument("--title", default="GTSAM trajectory", help="Plot title.")
    parser.add_argument(
        "--covariance-sigma",
        type=float,
        default=2.0,
        help="Sigma scale for covariance ellipses.",
    )
    return parser.parse_args()


def load_trajectories(path):
    trajectories = defaultdict(list)
    with open(path, newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        for row in reader:
            trajectories[row["name"]].append(
                {
                    "key": int(row["key"]),
                    "x": float(row["x"]),
                    "y": float(row["y"]),
                    "z": float(row["z"]) if row.get("z") else 0.0,
                    "theta": float(row["theta"]) if row["theta"] else None,
                    "cov_xx": parse_optional_float(row.get("cov_xx")),
                    "cov_xy": parse_optional_float(row.get("cov_xy")),
                    "cov_yy": parse_optional_float(row.get("cov_yy")),
                }
            )

    for points in trajectories.values():
        points.sort(key=lambda item: item["key"])
    return trajectories


def parse_optional_float(value):
    if value is None or value == "":
        return None
    return float(value)


def covariance_ellipse_params(cov_xx, cov_xy, cov_yy, sigma):
    trace = cov_xx + cov_yy
    diff = cov_xx - cov_yy
    root = math.sqrt(diff * diff + 4.0 * cov_xy * cov_xy)
    lambda1 = 0.5 * (trace + root)
    lambda2 = 0.5 * (trace - root)
    if lambda1 <= 0.0 or lambda2 <= 0.0:
        return None

    angle = 0.5 * math.atan2(2.0 * cov_xy, diff)
    width = 2.0 * sigma * math.sqrt(lambda1)
    height = 2.0 * sigma * math.sqrt(lambda2)
    return width, height, math.degrees(angle)


def plot_covariance(point, color, sigma):
    cov_xx = point["cov_xx"]
    cov_xy = point["cov_xy"]
    cov_yy = point["cov_yy"]
    if cov_xx is None or cov_xy is None or cov_yy is None:
        return

    params = covariance_ellipse_params(cov_xx, cov_xy, cov_yy, sigma)
    if params is None:
        return

    width, height, angle = params
    ellipse = Ellipse(
        xy=(point["x"], point["y"]),
        width=width,
        height=height,
        angle=angle,
        edgecolor=color,
        facecolor="none",
        linewidth=1.0,
        alpha=0.35,
        zorder=1,
    )
    plt.gca().add_patch(ellipse)


def style_for_name(name):
    normalized = name.lower()
    if normalized == "initial":
        return {
            "linestyle": "--",
            "marker": "o",
            "markerfacecolor": "none",
            "markersize": 8,
            "linewidth": 1.6,
            "zorder": 4,
        }
    if normalized == "result" or normalized == "optimized":
        return {
            "color": "green",
            "linestyle": "-",
            "marker": None,
            "markersize": 0,
            "linewidth": 1.8,
            "zorder": 5,
        }
    if normalized == "gps":
        return {
            "color": "red",
            "linestyle": "-",
            "marker": None,
            "markersize": 0,
            "linewidth": 1.4,
            "zorder": 3,
        }
    return {
        "linestyle": "-",
        "marker": "o",
        "markersize": 6,
        "linewidth": 1.8,
        "zorder": 4,
    }


def plot_trajectory(name, points, covariance_sigma):
    xs = [point["x"] for point in points]
    ys = [point["y"] for point in points]
    style = style_for_name(name)
    line = plt.plot(xs, ys, label=name, **style)[0]
    color = line.get_color()

    for point in points:
        plot_covariance(point, color, covariance_sigma)
        theta = point["theta"]
        if theta is None:
            continue
        arrow_len = 0.25
        plt.arrow(
            point["x"],
            point["y"],
            arrow_len * math.cos(theta),
            arrow_len * math.sin(theta),
            color=color,
            head_width=0.05,
            length_includes_head=True,
            alpha=0.8,
        )


def rmse(values):
    if not values:
        return 0.0
    return math.sqrt(sum(value * value for value in values) / len(values))


def component_errors(trajectories):
    optimized = trajectories.get("optimized") or trajectories.get("result")
    gps = trajectories.get("gps")
    if not optimized or not gps:
        return None

    gps_by_key = {point["key"]: point for point in gps}
    keys = []
    errors = {"x": [], "y": [], "z": []}
    for point in optimized:
        ref = gps_by_key.get(point["key"])
        if ref is None:
            continue
        keys.append(point["key"])
        errors["x"].append(point["x"] - ref["x"])
        errors["y"].append(point["y"] - ref["y"])
        errors["z"].append(point["z"] - ref["z"])

    if not keys:
        return None
    return keys, errors


def plot_error_curves(trajectories):
    errors_data = component_errors(trajectories)
    if errors_data is None:
        return False

    keys, errors = errors_data
    colors = {"x": "#2f6fdb", "y": "#d98c00", "z": "#6f42c1"}
    for axis_name in ("x", "y", "z"):
        values = errors[axis_name]
        plt.plot(
            keys,
            values,
            color=colors[axis_name],
            linewidth=1.4,
            label=f"{axis_name} error, RMSE={rmse(values):.3f} m",
        )

    plt.axhline(0.0, color="black", linewidth=0.8, alpha=0.5)
    plt.xlabel("key")
    plt.ylabel("error (m)")
    plt.grid(True, linestyle="--", linewidth=0.5, alpha=0.5)
    plt.legend()
    return True


def main():
    args = parse_args()
    trajectories = load_trajectories(args.csv)
    os.makedirs(os.path.dirname(args.out), exist_ok=True)

    has_errors = component_errors(trajectories) is not None
    if has_errors:
        plt.figure(figsize=(16, 12))
        plt.subplot(2, 1, 1)
    else:
        plt.figure(figsize=(14, 10))

    for name, points in trajectories.items():
        plot_trajectory(name, points, args.covariance_sigma)

    plt.title(args.title)
    plt.xlabel("x")
    plt.ylabel("y")
    plt.axis("equal")
    plt.grid(True, linestyle="--", linewidth=0.5, alpha=0.5)
    plt.legend()

    if has_errors:
        plt.subplot(2, 1, 2)
        plot_error_curves(trajectories)

    plt.tight_layout()
    plt.savefig(args.out, dpi=240)


if __name__ == "__main__":
    main()
