#!/usr/bin/env python3

import argparse
import math
import re
import struct
from dataclasses import dataclass
from pathlib import Path


WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3
LAYER_FILE_PATTERN = re.compile(r"lidar_metadata_layer_(\d+)\.meta$")
MAX_FRAME_GAP_NS = 500_000_000


@dataclass
class Pose:
    x: float
    y: float
    z: float


@dataclass
class FrameRecord:
    trip_id: str
    fid: str
    pose: Pose


def read_varint(data, offset):
    value = 0
    shift = 0
    while offset < len(data) and shift < 70:
        byte = data[offset]
        offset += 1
        value |= (byte & 0x7F) << shift
        if byte < 0x80:
            return value, offset
        shift += 7
    raise RuntimeError("invalid protobuf varint")


def iter_proto_fields(data):
    offset = 0
    while offset < len(data):
        key, offset = read_varint(data, offset)
        field_number = key >> 3
        wire_type = key & 0x07
        if wire_type == 0:
            value, offset = read_varint(data, offset)
        elif wire_type == 1:
            end = offset + 8
            if end > len(data):
                raise RuntimeError("truncated protobuf fixed64 field")
            value = data[offset:end]
            offset = end
        elif wire_type == 2:
            size, offset = read_varint(data, offset)
            end = offset + size
            if end > len(data):
                raise RuntimeError("truncated protobuf bytes field")
            value = data[offset:end]
            offset = end
        elif wire_type == 5:
            end = offset + 4
            if end > len(data):
                raise RuntimeError("truncated protobuf fixed32 field")
            value = data[offset:end]
            offset = end
        else:
            raise RuntimeError(f"unsupported protobuf wire type: {wire_type}")
        yield field_number, wire_type, value


def parse_pose(payload):
    coordinates = {}
    for field_number, wire_type, value in iter_proto_fields(payload):
        if field_number in (1, 2, 3) and wire_type == 1:
            coordinates[field_number] = struct.unpack("<d", value)[0]
    if not all(field in coordinates for field in (1, 2, 3)):
        return None
    return Pose(coordinates[1], coordinates[2], coordinates[3])


def parse_frame(payload):
    trip_id = ""
    fid = ""
    pose = None
    for field_number, wire_type, value in iter_proto_fields(payload):
        if field_number == 1 and wire_type == 2:
            trip_id = value.decode("utf-8", errors="replace")
        elif field_number == 2 and wire_type == 2:
            fid = value.decode("utf-8", errors="replace")
        elif field_number == 16 and wire_type == 2:
            pose = parse_pose(value)
    if pose is None:
        return None
    return FrameRecord(trip_id=trip_id, fid=fid, pose=pose)


def read_frame_metadata(path):
    frames = []
    with path.open("rb") as input_file:
        while True:
            length_bytes = input_file.read(4)
            if not length_bytes:
                break
            if len(length_bytes) != 4:
                raise RuntimeError(f"truncated frame length in {path}")
            payload_size = struct.unpack("<I", length_bytes)[0]
            payload = input_file.read(payload_size)
            if len(payload) != payload_size:
                raise RuntimeError(f"truncated frame payload in {path}")
            frame = parse_frame(payload)
            if frame is not None:
                frames.append(frame)
    return frames


def ecef_origin_to_lat_lon(origin):
    x, y, z = origin
    longitude = math.atan2(y, x)
    horizontal_distance = math.hypot(x, y)
    latitude = math.atan2(z, horizontal_distance * (1.0 - WGS84_E2))
    for _ in range(8):
        sin_latitude = math.sin(latitude)
        radius = WGS84_A / math.sqrt(
            1.0 - WGS84_E2 * sin_latitude * sin_latitude
        )
        latitude = math.atan2(
            z + WGS84_E2 * radius * sin_latitude,
            horizontal_distance,
        )
    return latitude, longitude


def make_ecef_to_enu(origin):
    latitude, longitude = ecef_origin_to_lat_lon(origin)
    sin_latitude = math.sin(latitude)
    cos_latitude = math.cos(latitude)
    sin_longitude = math.sin(longitude)
    cos_longitude = math.cos(longitude)

    def transform(point):
        dx = point[0] - origin[0]
        dy = point[1] - origin[1]
        dz = point[2] - origin[2]
        east = -sin_longitude * dx + cos_longitude * dy
        north = (
            -sin_latitude * cos_longitude * dx
            - sin_latitude * sin_longitude * dy
            + cos_latitude * dz
        )
        up = (
            cos_latitude * cos_longitude * dx
            + cos_latitude * sin_longitude * dy
            + sin_latitude * dz
        )
        return east, north, up

    return transform


def find_layer_metadata(input_dir):
    metadata = []
    for path in input_dir.iterdir():
        match = LAYER_FILE_PATTERN.fullmatch(path.name)
        if match and path.is_file():
            metadata.append((int(match.group(1)), path))
    metadata.sort(key=lambda item: item[0])
    if not metadata:
        raise RuntimeError(
            f"no lidar_metadata_layer_N.meta files found in {input_dir}"
        )
    return metadata


def frame_sort_key(frame):
    return frame.fid


def split_contiguous_frames(frames):
    groups = []
    current_group = []
    previous_timestamp = None
    for frame in frames:
        suffix = frame.fid.rsplit("_", 1)[-1]
        timestamp = int(suffix) if suffix.isdigit() else None
        if (current_group and timestamp is not None and
                previous_timestamp is not None and
                timestamp - previous_timestamp > MAX_FRAME_GAP_NS):
            groups.append(current_group)
            current_group = []
        current_group.append(frame)
        previous_timestamp = timestamp
    if current_group:
        groups.append(current_group)
    return groups


def visualize(input_dir, output_html, show, z_scale):
    layer_metadata = find_layer_metadata(input_dir)
    layers = []
    origin = None

    for layer_index, metadata_path in layer_metadata:
        frames = read_frame_metadata(metadata_path)
        if not frames:
            print(f"warning: no refined poses in {metadata_path}")
            continue
        if origin is None:
            pose = frames[0].pose
            origin = (pose.x, pose.y, pose.z)
        layers.append((layer_index, metadata_path, frames))
        print(f"loaded layer_{layer_index}: {len(frames)} frames")

    if not layers or origin is None:
        raise RuntimeError("no valid layer trajectories found")

    ecef_to_enu = make_ecef_to_enu(origin)

    try:
        import plotly.graph_objects as go
        from plotly.colors import qualitative
    except ImportError as error:
        raise RuntimeError(
            "Plotly is required; install it with: pip install plotly"
        ) from error

    figure = go.Figure()
    all_x = []
    all_y = []
    all_z = []

    for color_index, (layer_index, _, frames) in enumerate(layers):
        frames_by_trip = {}
        for frame in frames:
            trip_id = frame.trip_id or "unknown_trip"
            frames_by_trip.setdefault(trip_id, []).append(frame)

        color = qualitative.Plotly[color_index % len(qualitative.Plotly)]
        first_trace = True
        for trip_id, trip_frames in sorted(frames_by_trip.items()):
            trip_frames.sort(key=frame_sort_key)
            trip_point_count = 0
            for contiguous_frames in split_contiguous_frames(trip_frames):
                points = []
                for frame in contiguous_frames:
                    pose = frame.pose
                    east, north, up = ecef_to_enu((pose.x, pose.y, pose.z))
                    points.append((east, north, up * z_scale))

                xs = [point[0] for point in points]
                ys = [point[1] for point in points]
                zs = [point[2] for point in points]
                figure.add_trace(
                    go.Scatter3d(
                        x=xs,
                        y=ys,
                        z=zs,
                        mode="lines+markers" if len(points) > 1 else "markers",
                        name=f"layer_{layer_index}",
                        legendgroup=f"layer_{layer_index}",
                        showlegend=first_trace,
                        line=dict(color=color, width=4),
                        marker=dict(color=color, size=2),
                        customdata=[trip_id] * len(points),
                        hovertemplate=(
                            f"layer_{layer_index}<br>"
                            "trip=%{customdata}<br>"
                            "E=%{x:.2f} m<br>"
                            "N=%{y:.2f} m<br>"
                            "U=%{z:.2f} m<extra></extra>"
                        ),
                    )
                )
                first_trace = False
                trip_point_count += len(points)
                all_x.extend(xs)
                all_y.extend(ys)
                all_z.extend(zs)
            print(
                f"layer_{layer_index} trip={trip_id}: "
                f"plotted {trip_point_count} poses"
            )

    z_label = "Up (m)" if z_scale == 1.0 else f"Up × {z_scale:g}"

    x_range = max(all_x) - min(all_x)
    y_range = max(all_y) - min(all_y)
    z_range = max(all_z) - min(all_z)
    longest_range = max(x_range, y_range, z_range, 1.0)
    figure.update_layout(
        title="Road Layer Trajectories",
        scene=dict(
            xaxis_title="East (m)",
            yaxis_title="North (m)",
            zaxis_title=z_label,
            aspectmode="manual",
            aspectratio=dict(
                x=max(x_range / longest_range, 0.25),
                y=max(y_range / longest_range, 0.25),
                z=max(z_range / longest_range, 0.20),
            ),
            camera=dict(eye=dict(x=1.5, y=-1.5, z=0.9)),
        ),
        legend=dict(groupclick="togglegroup"),
        margin=dict(l=0, r=0, b=0, t=45),
    )

    output_html.parent.mkdir(parents=True, exist_ok=True)
    figure.write_html(
        str(output_html), include_plotlyjs=True, auto_open=show
    )
    print(f"saved interactive 3D visualization: {output_html}")


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Visualize lidar_metadata_layer_N.meta trajectories in local ENU."
        )
    )
    parser.add_argument(
        "input_dir",
        type=Path,
        help="Directory containing lidar_metadata_layer_N.meta files",
    )
    parser.add_argument(
        "--output_html",
        type=Path,
        help="Output HTML path; defaults to INPUT_DIR/layer_trajectories_3d.html",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Open the generated HTML in the default browser",
    )
    parser.add_argument(
        "--z_scale",
        type=float,
        default=1.0,
        help="Vertical exaggeration factor, default: 1.0",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    input_dir = args.input_dir.resolve()
    if not input_dir.is_dir():
        raise RuntimeError(f"input directory does not exist: {input_dir}")
    if not math.isfinite(args.z_scale) or args.z_scale <= 0.0:
        raise RuntimeError("--z_scale must be finite and positive")

    output_html = args.output_html
    if output_html is None:
        output_html = input_dir / "layer_trajectories_3d.html"
    visualize(input_dir, output_html.resolve(), args.show, args.z_scale)


if __name__ == "__main__":
    main()
