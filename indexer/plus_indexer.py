#!/usr/bin/env python3

import argparse
import bisect
from collections import Counter
import math
import struct
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path


NSEC_PER_SEC = 1000000000


def find_repo_root():
    current = Path(__file__).resolve()
    for parent in current.parents:
        if (parent / "mapping" / "protos" / "frame.proto").is_file():
            return parent
    raise RuntimeError(f"failed to find ADLabel repo root from: {current}")


def import_frame_pb2():
    repo_root = find_repo_root()
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


@dataclass(frozen=True)
class PoseSample:
    timestamp_ns: int
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float


class PoseTimeline:
    def __init__(self, samples):
        if not samples:
            raise RuntimeError("trajectory has no valid pose samples")
        self.samples = sorted(samples, key=lambda pose: pose.timestamp_ns)
        self.timestamps = [pose.timestamp_ns for pose in self.samples]

    def interpolate(self, timestamp_ns):
        index = bisect.bisect_left(self.timestamps, timestamp_ns)
        if index < len(self.samples) and self.timestamps[index] == timestamp_ns:
            return self.samples[index]
        if index == 0 or index == len(self.samples):
            return None
        return interpolate_pose(self.samples[index - 1], self.samples[index], timestamp_ns)


def timestamp_to_ns(value):
    value = float(value)
    if value > 1e12:
        return int(round(value))
    return int(round(value * NSEC_PER_SEC))


def normalize_quaternion(qx, qy, qz, qw):
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm == 0.0:
        return 0.0, 0.0, 0.0, 1.0
    return qx / norm, qy / norm, qz / norm, qw / norm


def slerp_quaternion(left, right, ratio):
    q1 = [left.qx, left.qy, left.qz, left.qw]
    q2 = [right.qx, right.qy, right.qz, right.qw]
    dot = sum(a * b for a, b in zip(q1, q2))
    if dot < 0.0:
        q2 = [-v for v in q2]
        dot = -dot

    if dot > 0.9995:
        q = [a + ratio * (b - a) for a, b in zip(q1, q2)]
        return normalize_quaternion(q[0], q[1], q[2], q[3])

    dot = max(-1.0, min(1.0, dot))
    theta_0 = math.acos(dot)
    theta = theta_0 * ratio
    sin_theta = math.sin(theta)
    sin_theta_0 = math.sin(theta_0)
    s0 = math.cos(theta) - dot * sin_theta / sin_theta_0
    s1 = sin_theta / sin_theta_0
    q = [s0 * a + s1 * b for a, b in zip(q1, q2)]
    return normalize_quaternion(q[0], q[1], q[2], q[3])


def interpolate_pose(left, right, timestamp_ns):
    span = right.timestamp_ns - left.timestamp_ns
    if span <= 0:
        return left
    ratio = (timestamp_ns - left.timestamp_ns) / span
    qx, qy, qz, qw = slerp_quaternion(left, right, ratio)
    return PoseSample(
        timestamp_ns=timestamp_ns,
        x=left.x + ratio * (right.x - left.x),
        y=left.y + ratio * (right.y - left.y),
        z=left.z + ratio * (right.z - left.z),
        qx=qx,
        qy=qy,
        qz=qz,
        qw=qw,
    )


def load_ecef_trajectory(path):
    samples = []
    with path.open("r", encoding="utf-8") as input_file:
        for line_no, line in enumerate(input_file, start=1):
            fields = line.strip().split()
            if not fields or not fields[0][0].isdigit():
                continue
            if len(fields) < 8:
                raise RuntimeError(f"trajectory line {line_no} has fewer than 8 columns: {path}")
            samples.append(
                PoseSample(
                    timestamp_ns=timestamp_to_ns(fields[0]),
                    x=float(fields[1]),
                    y=float(fields[2]),
                    z=float(fields[3]),
                    qx=float(fields[4]),
                    qy=float(fields[5]),
                    qz=float(fields[6]),
                    qw=float(fields[7]),
                )
            )
    return PoseTimeline(samples)


def read_meta_file(path):
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


def write_meta_file(path, frames):
    tmp_path = Path(str(path) + ".tmp")
    with tmp_path.open("wb") as output_file:
        for frame in frames:
            payload = frame.SerializeToString()
            output_file.write(struct.pack("<I", len(payload)))
            output_file.write(payload)
    tmp_path.replace(path)


def copy_pose(pose, pose_message):
    pose_message.x = pose.x
    pose_message.y = pose.y
    pose_message.z = pose.z
    pose_message.qx = pose.qx
    pose_message.qy = pose.qy
    pose_message.qz = pose.qz
    pose_message.qw = pose.qw


def process_dump_root(dump_root):
    meta_path = dump_root / "metadata" / "lidar" / "lidar_plusai_unified.meta"
    trajectory_path = dump_root / "offline_pose" / "gtsam_optim_rst_ecef.txt"
    cloud_dir = dump_root / "sensor_data" / "lidar" / "lidar_plusai_unified_compensated"
    lidar_nn_dir = dump_root / "pcd_segmentation" / "lidar_plusai_unified_compensated"

    if not meta_path.is_file():
        raise RuntimeError(f"lidar metadata file does not exist: {meta_path}")
    if not trajectory_path.is_file():
        raise RuntimeError(f"ECEF trajectory file does not exist: {trajectory_path}")

    if not cloud_dir.is_dir():
        raise RuntimeError(f"cloud directory does not exist: {cloud_dir}")
    if not lidar_nn_dir.is_dir():
        raise RuntimeError(f"lidar nn directory does not exist: {lidar_nn_dir}")

    local_data_reader_root = dump_root.parent
    timeline = load_ecef_trajectory(trajectory_path)
    cloud_files = {}
    for path in cloud_dir.iterdir():
        if path.is_file() and path.suffix == ".pcd" and path.stem.isdigit():
            cloud_files[int(path.stem)] = path

    lidar_nn_files = {}
    for path in lidar_nn_dir.iterdir():
        if path.is_file() and path.suffix == ".bin" and path.stem.isdigit():
            lidar_nn_files[int(path.stem)] = path
    frames = read_meta_file(meta_path)
    if not frames:
        raise RuntimeError(f"metadata has no frames: {meta_path}")

    sensor_counts = Counter()
    pose_updated = 0
    pose_skipped = 0
    cloud_updated = 0
    cloud_missing = 0
    lidar_nn_updated = 0
    lidar_nn_missing = 0

    for frame in frames:
        sensor_counts[frame.sensor_name or "<missing>"] += 1

        cloud_path = cloud_files.get(frame.timestamp_ns)
        if cloud_path is None:
            frame.ClearField("cloud_uri")
            cloud_missing += 1
        else:
            frame.cloud_uri = cloud_path.relative_to(local_data_reader_root).as_posix()
            cloud_updated += 1

        lidar_nn_path = lidar_nn_files.get(frame.timestamp_ns)
        if lidar_nn_path is None:
            frame.ClearField("lidar_nn_uri")
            lidar_nn_missing += 1
        else:
            frame.lidar_nn_uri = lidar_nn_path.relative_to(local_data_reader_root).as_posix()
            lidar_nn_updated += 1

        pose = timeline.interpolate(frame.timestamp_ns)
        if pose is None:
            pose_skipped += 1
            continue
        copy_pose(pose, frame.refined_pose_3d)
        pose_updated += 1

    write_meta_file(meta_path, frames)

    print("sensor_name frame counts:")
    for sensor_name, count in sorted(sensor_counts.items()):
        print(f"  {sensor_name}: {count}")
    print(f"cloud_uri: updated {cloud_updated}/{len(frames)}, missing {cloud_missing}")
    print(f"lidar_nn_uri: updated {lidar_nn_updated}/{len(frames)}, missing {lidar_nn_missing}")
    print(
        f"refined_pose_3d: updated {pose_updated}/{len(frames)}, "
        f"skipped {pose_skipped} outside trajectory range"
    )
    print(f"wrote metadata: {meta_path}")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Fill lidar_plusai_unified.meta Frame.refined_pose_3d from offline_pose/gtsam_optim_rst_ecef.txt."
    )
    parser.add_argument("dump_root", type=Path, help="Bag dump root directory")
    return parser.parse_args()


def main():
    args = parse_args()
    process_dump_root(args.dump_root)


if __name__ == "__main__":
    main()
