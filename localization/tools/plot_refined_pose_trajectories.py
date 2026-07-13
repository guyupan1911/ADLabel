#!/usr/bin/env python3

import argparse
import math
import struct
import subprocess
import sys
from pathlib import Path


WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3
COLORS = [
    "#e41a1c", "#377eb8", "#4daf4a", "#984ea3", "#ff7f00",
    "#a65628", "#f781bf", "#999999", "#66c2a5", "#fc8d62",
]


def import_frame_pb2():
    current = Path(__file__).resolve()
    repo_root = None
    for parent in current.parents:
        if (parent / "mapping" / "protos" / "frame.proto").is_file():
            repo_root = parent
            break
    if repo_root is None:
        raise RuntimeError(f"failed to find ADLabel repo root from: {current}")

    pyproto_dir = repo_root / "pyproto"
    frame_proto = repo_root / "mapping" / "protos" / "frame.proto"
    frame_pb2_path = pyproto_dir / "mapping" / "protos" / "frame_pb2.py"
    export_script = repo_root / "tools" / "export_pyproto.sh"

    needs_export = not frame_pb2_path.is_file()
    if frame_pb2_path.is_file():
        needs_export = frame_proto.stat().st_mtime > frame_pb2_path.stat().st_mtime
    if needs_export:
        if not export_script.is_file():
            raise ModuleNotFoundError(f"Python proto exporter not found: {export_script}")
        subprocess.run([str(export_script), str(pyproto_dir)], cwd=repo_root, check=True)

    sys.path.insert(0, str(pyproto_dir))
    from mapping.protos import frame_pb2
    return frame_pb2


frame_pb2 = import_frame_pb2()


def read_frame_metadata(path):
    frames = []
    with path.open("rb") as input_file:
        while True:
            length_bytes = input_file.read(4)
            if not length_bytes:
                break
            if len(length_bytes) != 4:
                raise RuntimeError(f"metadata has truncated frame length: {path}")
            length = struct.unpack("<I", length_bytes)[0]
            payload = input_file.read(length)
            if len(payload) != length:
                raise RuntimeError(f"metadata has truncated frame payload: {path}")
            frame = frame_pb2.Frame()
            frame.ParseFromString(payload)
            frames.append(frame)
    return frames


def ecef_to_lla(x, y, z):
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    lat = math.atan2(z, p * (1.0 - WGS84_E2))
    for _ in range(8):
        sin_lat = math.sin(lat)
        n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
        lat = math.atan2(z + WGS84_E2 * n * sin_lat, p)
    sin_lat = math.sin(lat)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    alt = p / math.cos(lat) - n
    return math.degrees(lat), math.degrees(lon), alt


def load_refined_pose_trajectories(metadata_paths):
    trajectories = []
    origin = None
    for metadata_path in metadata_paths:
        metadata_path = Path(metadata_path)
        frames = read_frame_metadata(metadata_path)
        local_points = []
        map_points = []
        trip_id = None
        for frame in frames:
            if trip_id is None and frame.trip_id:
                trip_id = frame.trip_id
            if not frame.HasField("refined_pose_3d"):
                continue
            pose = frame.refined_pose_3d
            if origin is None:
                origin = (pose.x, pose.y)
            local_points.append((pose.x - origin[0], pose.y - origin[1]))
            lat, lon, _ = ecef_to_lla(pose.x, pose.y, pose.z)
            map_points.append((lat, lon))

        if local_points:
            label = trip_id or metadata_path.stem
            trajectories.append((label, local_points, map_points, metadata_path))
        else:
            print(f"warning: no refined_pose_3d in {metadata_path}")

    if not trajectories:
        raise RuntimeError("no valid refined_pose_3d trajectory found")
    return trajectories


def save_png(trajectories, output_png):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(12, 10))
    for index, (label, local_points, _, metadata_path) in enumerate(trajectories):
        xs = [point[0] for point in local_points]
        ys = [point[1] for point in local_points]
        color = COLORS[index % len(COLORS)]
        ax.plot(xs, ys, linewidth=1.5, color=color, label=f"{label} ({len(local_points)})")
        ax.scatter(xs[0], ys[0], s=18, marker="o", color=color)
        ax.scatter(xs[-1], ys[-1], s=24, marker="x", color=color)
        print(f"{metadata_path}: plotted {len(local_points)} refined poses as {label}")

    ax.set_title("refined_pose_3d trajectories")
    ax.set_xlabel("x relative to first trajectory origin (m)")
    ax.set_ylabel("y relative to first trajectory origin (m)")
    ax.axis("equal")
    ax.grid(True, linewidth=0.3)
    ax.legend(loc="best")
    fig.tight_layout()

    output_png = Path(output_png)
    output_png.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_png, dpi=200)
    plt.close(fig)
    print(f"saved trajectory png: {output_png}")


def save_html(trajectories, output_html):
    import folium

    first_lat, first_lon = trajectories[0][2][0]
    map_view = folium.Map(location=[first_lat, first_lon], zoom_start=18)

    all_points = []
    for index, (label, _, map_points, _) in enumerate(trajectories):
        color = COLORS[index % len(COLORS)]
        folium.PolyLine(
            locations=map_points,
            color=color,
            weight=3,
            opacity=0.9,
            tooltip=f"{label} ({len(map_points)})",
        ).add_to(map_view)
        folium.CircleMarker(
            location=map_points[0],
            radius=4,
            color=color,
            fill=True,
            fill_color=color,
            fill_opacity=1.0,
            tooltip=f"{label} start",
        ).add_to(map_view)
        folium.Marker(
            location=map_points[-1],
            tooltip=f"{label} end",
        ).add_to(map_view)
        all_points.extend(map_points)

    if all_points:
        min_lat = min(point[0] for point in all_points)
        max_lat = max(point[0] for point in all_points)
        min_lon = min(point[1] for point in all_points)
        max_lon = max(point[1] for point in all_points)
        map_view.fit_bounds([[min_lat, min_lon], [max_lat, max_lon]])

    folium.LayerControl().add_to(map_view)
    output_html = Path(output_html)
    output_html.parent.mkdir(parents=True, exist_ok=True)
    map_view.save(str(output_html))
    print(f"saved trajectory html: {output_html}")


def plot_refined_pose_trajectories(metadata_paths, output_dir):
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    trajectories = load_refined_pose_trajectories(metadata_paths)
    save_png(trajectories, output_dir / "refined_pose_trajectories.png")
    save_html(trajectories, output_dir / "refined_pose_trajectories.html")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Plot refined_pose_3d trajectories from multiple Frame metadata files."
    )
    parser.add_argument("metadata", nargs="+", type=Path, help="Frame metadata files")
    parser.add_argument("--output_dir", required=True, type=Path, help="Directory to save PNG and HTML")
    return parser.parse_args()


def main():
    args = parse_args()
    plot_refined_pose_trajectories(args.metadata, args.output_dir)


if __name__ == "__main__":
    main()
