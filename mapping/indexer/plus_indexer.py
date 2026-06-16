#!/usr/bin/env python3

import argparse
import bisect
import json
import math
import struct
import sys
from dataclasses import dataclass
from pathlib import Path

try:
    from mapping.protos import camera_calibration_pb2
    from mapping.protos import frame_pb2
except ModuleNotFoundError:
    repo_root = Path(__file__).resolve().parents[2]
    pyproto_dir = repo_root / "pyproto"
    if pyproto_dir.exists():
        sys.path.insert(0, str(pyproto_dir))
        from mapping.protos import camera_calibration_pb2
        from mapping.protos import frame_pb2
    else:
        raise ModuleNotFoundError(
            "Python proto modules were not found. Run "
            "`tools/export_pyproto.sh` from the ADLabel workspace root, "
            "or set PYTHONPATH to an exported pyproto directory.") from None


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
        self.samples = sorted(samples, key=lambda pose: pose.timestamp_ns)
        self.timestamps = [pose.timestamp_ns for pose in self.samples]

    def get_pose(self, timestamp_ns, max_extrapolate_ns):
        if not self.samples:
            return None

        index = bisect.bisect_left(self.timestamps, timestamp_ns)
        if index < len(self.samples) and self.timestamps[index] == timestamp_ns:
            return self.samples[index]

        if index == 0:
            pose = self.samples[0]
            if abs(pose.timestamp_ns - timestamp_ns) <= max_extrapolate_ns:
                return pose
            return None
        if index == len(self.samples):
            pose = self.samples[-1]
            if abs(pose.timestamp_ns - timestamp_ns) <= max_extrapolate_ns:
                return pose
            return None

        return interpolate_pose(self.samples[index - 1],
                                self.samples[index],
                                timestamp_ns)


class UriIndex:
    def __init__(self, root, suffix):
        self.entries = []
        if root.exists():
            for path in root.glob(f"*{suffix}"):
                try:
                    timestamp_ns = int(path.stem)
                except ValueError:
                    continue
                self.entries.append((timestamp_ns, path))
        self.entries.sort(key=lambda item: item[0])
        self.timestamps = [item[0] for item in self.entries]

    def find(self, timestamp_ns, max_diff_ns):
        if not self.entries:
            return None

        index = bisect.bisect_left(self.timestamps, timestamp_ns)
        candidates = []
        if index < len(self.entries):
            candidates.append(self.entries[index])
        if index > 0:
            candidates.append(self.entries[index - 1])

        best_timestamp, best_path = min(
            candidates, key=lambda item: abs(item[0] - timestamp_ns))
        if abs(best_timestamp - timestamp_ns) > max_diff_ns:
            return None
        return best_path


def timestamp_to_ns(timestamp):
    value = float(timestamp)
    if value > 1e12:
        return int(round(value))
    return int(round(value * NSEC_PER_SEC))


def normalize_quaternion(qx, qy, qz, qw):
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm == 0.0:
        return 0.0, 0.0, 0.0, 1.0
    return qx / norm, qy / norm, qz / norm, qw / norm


def euler_to_quaternion(roll, pitch, yaw):
    half_roll = roll * 0.5
    half_pitch = pitch * 0.5
    half_yaw = yaw * 0.5

    cr = math.cos(half_roll)
    sr = math.sin(half_roll)
    cp = math.cos(half_pitch)
    sp = math.sin(half_pitch)
    cy = math.cos(half_yaw)
    sy = math.sin(half_yaw)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return normalize_quaternion(qx, qy, qz, qw)


def interpolate_pose(left, right, timestamp_ns):
    span = right.timestamp_ns - left.timestamp_ns
    if span <= 0:
        return left

    ratio = (timestamp_ns - left.timestamp_ns) / span
    x = left.x + ratio * (right.x - left.x)
    y = left.y + ratio * (right.y - left.y)
    z = left.z + ratio * (right.z - left.z)
    qx = left.qx + ratio * (right.qx - left.qx)
    qy = left.qy + ratio * (right.qy - left.qy)
    qz = left.qz + ratio * (right.qz - left.qz)
    qw = left.qw + ratio * (right.qw - left.qw)
    qx, qy, qz, qw = normalize_quaternion(qx, qy, qz, qw)
    return PoseSample(timestamp_ns, x, y, z, qx, qy, qz, qw)


def copy_pose(pose, pose_message):
    pose_message.x = pose.x
    pose_message.y = pose.y
    pose_message.z = pose.z
    pose_message.qw = pose.qw
    pose_message.qx = pose.qx
    pose_message.qy = pose.qy
    pose_message.qz = pose.qz


def pose_from_sensor2ego(sensor2ego):
    qx, qy, qz, qw = euler_to_quaternion(
        float(sensor2ego.get("roll", 0.0)),
        float(sensor2ego.get("pitch", 0.0)),
        float(sensor2ego.get("yaw", 0.0)),
    )
    return PoseSample(
        timestamp_ns=0,
        x=float(sensor2ego.get("x", 0.0)),
        y=float(sensor2ego.get("y", 0.0)),
        z=float(sensor2ego.get("z", 0.0)),
        qx=qx,
        qy=qy,
        qz=qz,
        qw=qw,
    )


def load_raw_clip(path):
    with path.open() as input_file:
        return json.load(input_file)


def load_calibrations(path):
    with path.open() as input_file:
        calibration = json.load(input_file)
    sensors = calibration["sensor_calibration"]["sensor_list"]
    if isinstance(sensors, dict):
        normalized = {}
        for sensor_id, sensor in sensors.items():
            sensor = dict(sensor)
            sensor.setdefault("sensor_id", sensor_id)
            normalized[sensor["sensor_id"]] = sensor
        return normalized
    return {sensor["sensor_id"]: sensor for sensor in sensors}


def load_pose_timeline(path):
    samples = []
    with path.open() as input_file:
        for line in input_file:
            fields = line.strip().split()
            if not fields or not fields[0][0].isdigit():
                continue
            if len(fields) < 8:
                continue
            samples.append(PoseSample(
                timestamp_ns=timestamp_to_ns(fields[0]),
                x=float(fields[1]),
                y=float(fields[2]),
                z=float(fields[3]),
                qx=float(fields[4]),
                qy=float(fields[5]),
                qz=float(fields[6]),
                qw=float(fields[7]),
            ))
    return PoseTimeline(samples)


def make_frame_id(sensor_name, frame_head):
    return f"{sensor_name}_{int(frame_head['frame_id']):06d}"


def fill_common_frame(frame, trip_id, sensor_name, frame_head, sensor_type,
                      calibration, lio_timeline, gnss_timeline,
                      max_pose_extrapolate_ns):
    timestamp_ns = timestamp_to_ns(frame_head["frame_timestamp"])
    frame.trip_id = trip_id
    frame.fid = make_frame_id(sensor_name, frame_head)
    frame.timestamp_ns = timestamp_ns
    frame.sensor_type = sensor_type
    frame.sensor_name = sensor_name

    sensor_to_ego = pose_from_sensor2ego(calibration.get("sensor2ego", {}))
    copy_pose(sensor_to_ego, frame.sensor_to_imu_extrinsic)

    lio_pose = lio_timeline.get_pose(timestamp_ns, max_pose_extrapolate_ns)
    if lio_pose is not None:
        copy_pose(lio_pose, frame.lio_pose_3d)

    gnss_pose = gnss_timeline.get_pose(timestamp_ns, max_pose_extrapolate_ns)
    if gnss_pose is not None:
        copy_pose(gnss_pose, frame.gnss_pose_3d)


def fill_prev_next(frames):
    for index, frame in enumerate(frames):
        if index > 0:
            frame.prev_id = frames[index - 1].fid
        if index + 1 < len(frames):
            frame.next_id = frames[index + 1].fid


def fill_camera_calibration(frame, calibration):
    sensor_type = calibration.get("sensor_type", "")
    if sensor_type == "fisheye":
        frame.camera_calibration.camera_type = (
            camera_calibration_pb2.CAMERA_TYPE_FISHEYE)
    elif sensor_type == "pinhole":
        frame.camera_calibration.camera_type = (
            camera_calibration_pb2.CAMERA_TYPE_PINHOLE)
    else:
        frame.camera_calibration.camera_type = (
            camera_calibration_pb2.CAMERA_TYPE_UNKNOWN)

    frame.camera_calibration.intrinsics.extend(
        float(value) for value in calibration.get("camera_intrinsic", []))
    frame.camera_calibration.distortion_coefficients.extend(
        float(value)
        for value in calibration.get("camera_distortion_coefficients", []))

    image_size = calibration.get("image_size", {})
    if "width" in image_size:
        frame.camera_calibration.width = int(image_size["width"])
    if "height" in image_size:
        frame.camera_calibration.height = int(image_size["height"])


def relative_uri(path, clip_root):
    return path.relative_to(clip_root).as_posix()


def build_lidar_frames(raw_clip, clip_root, calibrations, lio_timeline,
                       gnss_timeline, args):
    sensor_name = "lidar_plusai_unified"
    calibration = calibrations[sensor_name]
    trip_id = raw_clip["clip_head"]["clip_id"]
    detection_index = UriIndex(
        clip_root / "pcd_segmentation" / sensor_name, ".bin")

    frames = []
    missing_detection = 0
    filled_detection = 0
    for raw_frame in raw_clip["reference_sensor"]["frame_list"]:
        frame_head = raw_frame["frame_head"]
        frame = frame_pb2.Frame()
        fill_common_frame(
            frame=frame,
            trip_id=trip_id,
            sensor_name=sensor_name,
            frame_head=frame_head,
            sensor_type=frame_pb2.Frame.LIDAR,
            calibration=calibration,
            lio_timeline=lio_timeline,
            gnss_timeline=gnss_timeline,
            max_pose_extrapolate_ns=args.max_pose_extrapolate_ms * 1000000,
        )
        frame.cloud_uri = frame_head["frame_name"]

        detection_path = detection_index.find(
            frame.timestamp_ns, args.max_uri_match_ms * 1000000)
        if detection_path is not None:
            frame.lidar_object_detection_uri = relative_uri(
                detection_path, clip_root)
            filled_detection += 1
        else:
            missing_detection += 1
        frames.append(frame)

    fill_prev_next(frames)
    return frames, {
        "frames": len(frames),
        "lidar_object_detection_uri_filled": filled_detection,
        "missing_lidar_detection": missing_detection,
    }


def build_camera_frames(raw_clip, clip_root, calibrations, lio_timeline,
                        gnss_timeline, args):
    trip_id = raw_clip["clip_head"]["clip_id"]
    camera_segments = {
        "J6_front_wide_camera": UriIndex(
            clip_root / "image_segmentation" / "J6_front_wide_camera", ".png")
    }

    output = {}
    stats = {}
    for sensor in raw_clip["sensor_list"]:
        sensor_head = sensor["sensor_head"]
        if sensor_head.get("source_type") != "image":
            continue

        sensor_name = sensor_head["sensor_id"]
        calibration = calibrations[sensor_name]
        frames = []
        missing_segmentation = 0
        filled_segmentation = 0

        for raw_frame in sensor["frame_list"]:
            frame_head = raw_frame["frame_head"]
            frame = frame_pb2.Frame()
            fill_common_frame(
                frame=frame,
                trip_id=trip_id,
                sensor_name=sensor_name,
                frame_head=frame_head,
                sensor_type=frame_pb2.Frame.CAMERA,
                calibration=calibration,
                lio_timeline=lio_timeline,
                gnss_timeline=gnss_timeline,
                max_pose_extrapolate_ns=args.max_pose_extrapolate_ms * 1000000,
            )
            frame.camera_image_uri = frame_head["frame_name"]
            fill_camera_calibration(frame, calibration)

            segment_index = camera_segments.get(sensor_name)
            if segment_index is not None:
                segmentation_path = segment_index.find(
                    frame.timestamp_ns, args.max_uri_match_ms * 1000000)
                if segmentation_path is not None:
                    frame.camera_segmentation_uri = relative_uri(
                        segmentation_path, clip_root)
                    filled_segmentation += 1
                else:
                    missing_segmentation += 1

            frames.append(frame)

        fill_prev_next(frames)
        output[sensor_name] = frames
        stats[sensor_name] = {
            "frames": len(frames),
            "camera_segmentation_expected": sensor_name in camera_segments,
            "camera_segmentation_uri_filled": filled_segmentation,
            "missing_camera_segmentation": missing_segmentation,
        }

    return output, stats


def write_meta_file(path, frames):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("wb") as output_file:
        for frame in frames:
            payload = frame.SerializeToString()
            output_file.write(struct.pack("<I", len(payload)))
            output_file.write(payload)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Create ADLabel Frame metadata from a Plus raw clip.")
    parser.add_argument("--clip_root", required=True,
                        help="Root directory of the Plus clip.")
    parser.add_argument("--raw_clip", default="raw_clip_51.json",
                        help="raw_clip JSON path, relative to clip_root unless absolute.")
    parser.add_argument("--output_root", default=None,
                        help="Output root. Defaults to clip_root.")
    parser.add_argument("--calibration", default="sensor_calibration.json",
                        help="Calibration JSON path, relative to clip_root unless absolute.")
    parser.add_argument("--lio_traj", default="offline_pose/lidar_odom_traj.txt",
                        help="Pure LIO/lidar odom trajectory path.")
    parser.add_argument(
        "--gnss_traj",
        default="offline_pose/pose_optimizer_fusion_output_traj.txt",
        help="GTSAM LIO+GNSS fusion trajectory path.")
    parser.add_argument("--max_pose_extrapolate_ms", type=float, default=200.0)
    parser.add_argument("--max_uri_match_ms", type=float, default=60.0)
    return parser.parse_args()


def resolve_path(clip_root, path):
    resolved = Path(path)
    if resolved.is_absolute():
        return resolved
    return clip_root / resolved


def main():
    args = parse_args()
    clip_root = Path(args.clip_root).resolve()
    output_root = Path(args.output_root).resolve() if args.output_root else clip_root

    raw_clip = load_raw_clip(resolve_path(clip_root, args.raw_clip))
    calibrations = load_calibrations(resolve_path(clip_root, args.calibration))
    lio_timeline = load_pose_timeline(resolve_path(clip_root, args.lio_traj))
    gnss_timeline = load_pose_timeline(resolve_path(clip_root, args.gnss_traj))

    lidar_frames, lidar_stats = build_lidar_frames(
        raw_clip, clip_root, calibrations, lio_timeline, gnss_timeline, args)
    write_meta_file(
        output_root / "metadata" / "lidar" / "lidar_plusai_unified.meta",
        lidar_frames)

    camera_frames_by_sensor, camera_stats = build_camera_frames(
        raw_clip, clip_root, calibrations, lio_timeline, gnss_timeline, args)
    for sensor_name, frames in camera_frames_by_sensor.items():
        write_meta_file(
            output_root / "metadata" / "camera" / f"{sensor_name}.meta",
            frames)

    print(f"wrote lidar_plusai_unified frames: {len(lidar_frames)}")
    print(f"lidar stats: {lidar_stats}")
    for sensor_name, stats in sorted(camera_stats.items()):
        print(f"camera {sensor_name}: {stats}")


if __name__ == "__main__":
    main()
