#!/usr/bin/env python3

import argparse
import bisect
import math
import struct
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path


def import_frame_pb2():
    try:
        from mapping.protos import frame_pb2
        return frame_pb2
    except ModuleNotFoundError:
        pass

    repo_root = Path(__file__).resolve().parents[2]
    pyproto_dir = repo_root / "pyproto"
    export_script = repo_root / "tools" / "export_pyproto.sh"
    if not (pyproto_dir / "mapping" / "protos" / "frame_pb2.py").is_file():
        if not export_script.is_file():
            raise ModuleNotFoundError(f"Python proto exporter not found: {export_script}") from None
        subprocess.run([str(export_script), str(pyproto_dir)], cwd=repo_root, check=True)

    sys.path.insert(0, str(pyproto_dir))
    from mapping.protos import frame_pb2
    return frame_pb2


frame_pb2 = import_frame_pb2()


NSEC_PER_SEC = 1000000000


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
    x = left.x + ratio * (right.x - left.x)
    y = left.y + ratio * (right.y - left.y)
    z = left.z + ratio * (right.z - left.z)
    qx, qy, qz, qw = slerp_quaternion(left, right, ratio)
    return PoseSample(timestamp_ns, x, y, z, qx, qy, qz, qw)


def load_trajectory(path):
    samples = []
    with Path(path).open("r", encoding="utf-8") as input_file:
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
    with Path(path).open("rb") as input_file:
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


def update_refined_pose(metadata_path, trajectory_path):
    timeline = load_trajectory(trajectory_path)
    frames = read_meta_file(metadata_path)
    if not frames:
        raise RuntimeError(f"metadata has no frames: {metadata_path}")

    updated = 0
    missing = 0
    for frame in frames:
        pose = timeline.interpolate(frame.timestamp_ns)
        if pose is None:
            missing += 1
            continue
        copy_pose(pose, frame.refined_pose_3d)
        updated += 1

    write_meta_file(metadata_path, frames)
    print(
        f"updated refined_pose_3d for {updated} frames, "
        f"skipped {missing} frames outside trajectory time range: {metadata_path}"
    )


def parse_args():
    parser = argparse.ArgumentParser(
        description="Fill Frame.refined_pose_3d in a metadata file from a vehicle trajectory."
    )
    parser.add_argument("trajectory", type=Path, help="Vehicle trajectory txt file")
    parser.add_argument("metadata", type=Path, help="Length-prefixed Frame metadata file to update in place")
    return parser.parse_args()


def main():
    args = parse_args()
    update_refined_pose(args.metadata, args.trajectory)


if __name__ == "__main__":
    main()
