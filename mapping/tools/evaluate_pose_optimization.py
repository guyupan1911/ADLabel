#!/usr/bin/env python3

import argparse
import csv
import math
from pathlib import Path

import numpy as np

try:
    import matplotlib.pyplot as plt
except ImportError:
    plt = None


def load_csv(path):
    rows = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append({k: float(v) for k, v in row.items()})
    if not rows:
        raise RuntimeError(f"empty csv: {path}")
    return rows


def load_factor_csv(path):
    rows = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(
                {
                    "type": row["type"],
                    "from": int(row["from"]),
                    "to": int(row["to"]),
                }
            )
    if not rows:
        raise RuntimeError(f"empty factor csv: {path}")
    return rows


def values(rows, key):
    return np.array([row[key] for row in rows], dtype=np.float64)


def stats(name, err):
    return {
        "name": name,
        "mean": float(np.mean(err)),
        "mean_abs": float(np.mean(np.abs(err))),
        "rmse": float(np.sqrt(np.mean(err * err))),
        "median": float(np.median(err)),
        "median_abs": float(np.median(np.abs(err))),
        "p95": float(np.percentile(err, 95)),
        "p95_abs": float(np.percentile(np.abs(err), 95)),
        "max": float(np.max(err)),
        "max_abs": float(np.max(np.abs(err))),
    }


def write_stats(path, stat_rows):
    with open(path, "w") as f:
        f.write("name,mean,mean_abs,rmse,median,median_abs,p95,p95_abs,max,max_abs\n")
        for row in stat_rows:
            f.write(
                f"{row['name']},{row['mean']:.6f},{row['mean_abs']:.6f},"
                f"{row['rmse']:.6f},{row['median']:.6f},{row['median_abs']:.6f},"
                f"{row['p95']:.6f},{row['p95_abs']:.6f},{row['max']:.6f},"
                f"{row['max_abs']:.6f}\n"
            )
    for row in stat_rows:
        print(
            f"{row['name']}: mean={row['mean']:.4f}, mean_abs={row['mean_abs']:.4f}, "
            f"rmse={row['rmse']:.4f}, p95_abs={row['p95_abs']:.4f}, "
            f"max_abs={row['max_abs']:.4f}"
        )


def make_stat_rows(rows):
    angle_scale = 180.0 / math.pi
    stat_rows = [
        stats("optimized_pos_norm_m", values(rows, "optimized_pos_error_m")),
    ]
    for axis in ["x", "y", "z"]:
        stat_rows.append(stats(f"optimized_{axis}_m", values(rows, f"optimized_error_{axis}_m")))
    for axis in ["roll", "pitch", "yaw"]:
        err_deg = values(rows, f"optimized_error_{axis}_rad") * angle_scale
        stat_rows.append(stats(f"optimized_{axis}_deg", err_deg))
    return stat_rows


def plot_component_errors(path, rows):
    if plt is None:
        return
    t = values(rows, "timestamp_sec")
    t = t - t[0]
    fig, axes = plt.subplots(2, 3, figsize=(15, 8), sharex=True)

    for ax, axis in zip(axes[0], ["x", "y", "z"]):
        ax.plot(t, values(rows, f"optimized_error_{axis}_m"), label="optimized")
        ax.set_title(f"{axis} error")
        ax.set_ylabel("[m]")
        ax.grid(True, linestyle="--", alpha=0.4)

    for ax, axis in zip(axes[1], ["roll", "pitch", "yaw"]):
        ax.plot(
            t,
            values(rows, f"optimized_error_{axis}_rad") * 180.0 / math.pi,
            label="optimized",
        )
        ax.set_title(f"{axis} error")
        ax.set_xlabel("time [s]")
        ax.set_ylabel("[deg]")
        ax.grid(True, linestyle="--", alpha=0.4)

    axes[0][0].legend()
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def plot_xy(path, rows, graph_nodes=None):
    if plt is None:
        return
    plt.figure(figsize=(10, 8))
    plt.plot(values(rows, "gnss_x"), values(rows, "gnss_y"), label="gnss", linewidth=2)
    plt.plot(
        values(rows, "optimized_x"),
        values(rows, "optimized_y"),
        label="optimized",
        linewidth=1,
    )
    if graph_nodes is not None:
        opt_x = values(graph_nodes, "optimized_x")
        opt_y = values(graph_nodes, "optimized_y")
        gnss_x = values(graph_nodes, "gnss_x")
        gnss_y = values(graph_nodes, "gnss_y")
        has_gps = values(graph_nodes, "has_gnss_factor") > 0.5
        for i in np.where(has_gps)[0]:
            plt.plot(
                [opt_x[i], gnss_x[i]],
                [opt_y[i], gnss_y[i]],
                color="tab:red",
                alpha=0.45,
                linewidth=0.9,
            )
        plt.scatter(
            gnss_x[has_gps],
            gnss_y[has_gps],
            s=58,
            marker="x",
            color="tab:red",
            linewidths=1.6,
            label="GPSFactor target",
        )
        plt.scatter(
            opt_x[has_gps],
            opt_y[has_gps],
            s=42,
            facecolors="none",
            edgecolors="tab:orange",
            linewidths=1.2,
            label="nodes with GPSFactor",
        )
    plt.axis("equal")
    plt.grid(True, linestyle="--", alpha=0.4)
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.legend()
    plt.tight_layout()
    plt.savefig(path, dpi=160)
    plt.close()


def plot_graph(path, nodes, factors):
    if plt is None:
        return

    opt_x = values(nodes, "optimized_x")
    opt_y = values(nodes, "optimized_y")
    lio_x = values(nodes, "aligned_lio_x")
    lio_y = values(nodes, "aligned_lio_y")
    gnss_x = values(nodes, "gnss_x")
    gnss_y = values(nodes, "gnss_y")
    has_gps = values(nodes, "has_gnss_factor") > 0.5

    fig, ax = plt.subplots(figsize=(12, 10))
    ax.plot(lio_x, lio_y, color="0.72", linewidth=1.0, label="aligned LIO")
    ax.plot(opt_x, opt_y, color="tab:blue", linewidth=1.4, label="optimized")
    ax.scatter(opt_x, opt_y, s=8, color="tab:blue", alpha=0.45, label="nodes")

    lio_factors = [f for f in factors if f["type"] == "lio_between"]
    stride = max(1, len(lio_factors) // 350)
    for factor_index, factor in enumerate(lio_factors):
        if factor_index % stride != 0:
            continue
        i = factor["from"]
        j = factor["to"]
        if i >= len(nodes) or j >= len(nodes):
            continue
        ax.plot(
            [opt_x[i], opt_x[j]],
            [opt_y[i], opt_y[j]],
            color="tab:green",
            alpha=0.16,
            linewidth=0.6,
        )

    gps_indices = np.where(has_gps)[0]
    for i in gps_indices:
        ax.plot(
            [opt_x[i], gnss_x[i]],
            [opt_y[i], gnss_y[i]],
            color="tab:red",
            alpha=0.35,
            linewidth=0.9,
        )

    ax.scatter(
        gnss_x[has_gps],
        gnss_y[has_gps],
        s=58,
        marker="x",
        color="tab:red",
        linewidths=1.6,
        label="GPSFactor target",
    )
    ax.scatter(
        opt_x[has_gps],
        opt_y[has_gps],
        s=42,
        facecolors="none",
        edgecolors="tab:orange",
        linewidths=1.2,
        label="nodes with GPSFactor",
    )

    ax.axis("equal")
    ax.grid(True, linestyle="--", alpha=0.35)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Pose graph nodes and factors")
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(path, dpi=180)
    plt.close(fig)


def plot_errors(path, rows):
    if plt is None:
        return
    t = values(rows, "timestamp_sec")
    t = t - t[0]
    opt_pos = values(rows, "optimized_pos_error_m")
    opt_yaw = np.abs(values(rows, "optimized_yaw_error_rad")) * 180.0 / math.pi

    fig, axes = plt.subplots(2, 1, figsize=(11, 8), sharex=True)
    axes[0].plot(t, opt_pos, label="optimized")
    axes[0].set_ylabel("position error [m]")
    axes[0].grid(True, linestyle="--", alpha=0.4)
    axes[0].legend()

    axes[1].plot(t, opt_yaw, label="optimized")
    axes[1].set_xlabel("time [s]")
    axes[1].set_ylabel("yaw error [deg]")
    axes[1].grid(True, linestyle="--", alpha=0.4)
    axes[1].legend()

    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", required=True, help="pose_optimization_eval.csv from pose_optimize_cli")
    parser.add_argument("--graph_nodes_csv", default="", help="pose_graph_nodes.csv from pose_optimize_cli")
    parser.add_argument(
        "--graph_factors_csv", default="", help="pose_graph_factors.csv from pose_optimize_cli"
    )
    parser.add_argument("--output_dir", default="", help="Directory for stats and plots")
    args = parser.parse_args()

    csv_path = Path(args.csv)
    output_dir = Path(args.output_dir) if args.output_dir else csv_path.parent
    output_dir.mkdir(parents=True, exist_ok=True)

    rows = load_csv(csv_path)
    write_stats(output_dir / "pose_error_stats.csv", make_stat_rows(rows))
    if plt is None:
        print("matplotlib is not installed; skipped plot generation")
        return

    graph_nodes_csv = Path(args.graph_nodes_csv) if args.graph_nodes_csv else csv_path.parent / "pose_graph_nodes.csv"
    graph_factors_csv = (
        Path(args.graph_factors_csv) if args.graph_factors_csv else csv_path.parent / "pose_graph_factors.csv"
    )
    graph_nodes = load_csv(graph_nodes_csv) if graph_nodes_csv.is_file() else None
    graph_factors = load_factor_csv(graph_factors_csv) if graph_factors_csv.is_file() else None

    plot_xy(output_dir / "pose_trajectory_xy.png", rows, graph_nodes)
    plot_errors(output_dir / "pose_errors.png", rows)
    plot_component_errors(output_dir / "pose_component_errors.png", rows)

    if graph_nodes is not None and graph_factors is not None:
        plot_graph(output_dir / "pose_graph_xy.png", graph_nodes, graph_factors)
    else:
        print(
            "graph csv files not found; skipped graph plot: "
            f"{graph_nodes_csv}, {graph_factors_csv}"
        )


if __name__ == "__main__":
    main()
