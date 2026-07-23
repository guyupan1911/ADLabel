#include <sys/stat.h>
#include <sys/types.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include "mapping/common/file.h"
#include "mapping/protos/frame.pb.h"
#include "mapping/protos/localization.pb.h"
#include "mapping/protos/pose.pb.h"

DEFINE_string(lidar_metadata, "",
              "Input lidar metadata file with lio_pose_3d.");
DEFINE_string(localization_bin, "",
              "Length-prefixed LocalizationEstimation bin file.");
DEFINE_string(
    output_eval_csv, "data/pose_optimization_eval.csv",
    "CSV containing timestamp-aligned gnss, aligned_lio, and optimized poses.");
DEFINE_string(output_graph_nodes_csv, "data/pose_graph_nodes.csv",
              "CSV containing graph nodes for visualization.");
DEFINE_string(output_graph_factors_csv, "data/pose_graph_factors.csv",
              "CSV containing graph factors for visualization.");

DEFINE_double(max_gnss_gap_sec, 0.2,
              "Max allowed time distance to GNSS interpolation bounds.");
DEFINE_double(lio_translation_sigma, 0.02,
              "LIO between-factor translation sigma in meters.");
DEFINE_double(lio_rotation_sigma_rad, 0.003,
              "LIO between-factor rotation sigma in radians.");
DEFINE_double(gnss_xy_sigma, 5.0, "GNSS position x/y sigma in meters.");
DEFINE_double(gnss_z_sigma, 10.0, "GNSS position z sigma in meters.");
DEFINE_double(min_gnss_factor_distance_m, 20.0,
              "Minimum accumulated LIO trajectory distance between RTK fixed "
              "GNSS factors in "
              "meters. Set to 0 to disable distance downsampling.");
DEFINE_bool(use_local_coordinate, true,
            "Optimize in a local coordinate frame whose origin is the first "
            "RTK fixed GNSS pose.");
DEFINE_bool(add_first_prior, false,
            "Add a first-pose prior for ablation experiments.");
DEFINE_double(first_prior_translation_sigma, 0.05,
              "First pose prior translation sigma in meters.");
DEFINE_double(first_prior_rotation_sigma_rad, 0.02,
              "First pose prior rotation sigma in radians.");

namespace adlabel {
namespace mapping {
namespace {

using gtsam::symbol_shorthand::X;
constexpr double kPi = 3.14159265358979323846;

struct OptimizerNode {
  std::size_t frame_index = 0;
  double timestamp_sec = 0.0;
  gtsam::Pose3 lio_pose;
  gtsam::Pose3 aligned_lio_pose;
  gtsam::Pose3 gnss_pose;
  bool has_gnss_factor = false;
};

struct RpeMetrics {
  bool valid = false;
  double translation_error_m = std::numeric_limits<double>::quiet_NaN();
  double rotation_error_rad = std::numeric_limits<double>::quiet_NaN();
  double yaw_error_rad = std::numeric_limits<double>::quiet_NaN();
};

struct GraphFactorRecord {
  std::string type;
  std::size_t from = 0;
  std::size_t to = 0;
};

gtsam::Pose3 ToGtsamPose(const Pose3DMessage& pose) {
  Eigen::Quaterniond q(pose.qw(), pose.qx(), pose.qy(), pose.qz());
  q.normalize();
  return gtsam::Pose3(gtsam::Rot3::Quaternion(q.w(), q.x(), q.y(), q.z()),
                      gtsam::Point3(pose.x(), pose.y(), pose.z()));
}

void FillPoseMessage(const gtsam::Pose3& pose, Pose3DMessage* msg) {
  const gtsam::Point3 t = pose.translation();
  const Eigen::Quaterniond q = pose.rotation().toQuaternion().normalized();
  msg->set_x(t.x());
  msg->set_y(t.y());
  msg->set_z(t.z());
  msg->set_qw(q.w());
  msg->set_qx(q.x());
  msg->set_qy(q.y());
  msg->set_qz(q.z());
}

double FrameTimestampSec(const Frame& frame) {
  return static_cast<double>(frame.timestamp_ns()) * 1e-9;
}

double GnssTimestampSec(const LocalizationEstimation& gnss) {
  return gnss.header().timestamp_msec() * 1e-3;
}

gtsam::Pose3 GnssToPose(const LocalizationEstimation& gnss) {
  const auto& pos = gnss.position();
  const auto& ori = gnss.orientation();
  Eigen::Quaterniond q(ori.qw(), ori.qx(), ori.qy(), ori.qz());
  q.normalize();
  return gtsam::Pose3(gtsam::Rot3::Quaternion(q.w(), q.x(), q.y(), q.z()),
                      gtsam::Point3(pos.x(), pos.y(), pos.z()));
}

gtsam::Pose3 InterpolatePose(const LocalizationEstimation& before,
                             const LocalizationEstimation& after,
                             double timestamp_sec) {
  const double t0 = GnssTimestampSec(before);
  const double t1 = GnssTimestampSec(after);
  double ratio = 0.0;
  if (std::abs(t1 - t0) > 1e-9) {
    ratio = (timestamp_sec - t0) / (t1 - t0);
  }
  ratio = std::max(0.0, std::min(1.0, ratio));

  const auto& p0 = before.position();
  const auto& p1 = after.position();
  const Eigen::Vector3d t =
      (1.0 - ratio) * Eigen::Vector3d(p0.x(), p0.y(), p0.z()) +
      ratio * Eigen::Vector3d(p1.x(), p1.y(), p1.z());

  const auto& o0 = before.orientation();
  const auto& o1 = after.orientation();
  Eigen::Quaterniond q0(o0.qw(), o0.qx(), o0.qy(), o0.qz());
  Eigen::Quaterniond q1(o1.qw(), o1.qx(), o1.qy(), o1.qz());
  q0.normalize();
  q1.normalize();
  const Eigen::Quaterniond q = q0.slerp(ratio, q1).normalized();

  return gtsam::Pose3(gtsam::Rot3::Quaternion(q.w(), q.x(), q.y(), q.z()),
                      gtsam::Point3(t.x(), t.y(), t.z()));
}

bool InterpolateGnssPose(const std::vector<LocalizationEstimation>& gnss_msgs,
                         double timestamp_sec, double max_gap_sec,
                         gtsam::Pose3* pose) {
  if (gnss_msgs.empty()) {
    return false;
  }

  auto iter =
      std::lower_bound(gnss_msgs.begin(), gnss_msgs.end(), timestamp_sec,
                       [](const LocalizationEstimation& gnss, double ts) {
                         return GnssTimestampSec(gnss) < ts;
                       });

  if (iter == gnss_msgs.begin()) {
    const double diff = std::abs(GnssTimestampSec(*iter) - timestamp_sec);
    if (diff > max_gap_sec) {
      return false;
    }
    *pose = GnssToPose(*iter);
    return true;
  }

  if (iter == gnss_msgs.end()) {
    const auto& last = gnss_msgs.back();
    const double diff = std::abs(timestamp_sec - GnssTimestampSec(last));
    if (diff > max_gap_sec) {
      return false;
    }
    *pose = GnssToPose(last);
    return true;
  }

  const auto& after = *iter;
  const auto& before = *std::prev(iter);
  if (timestamp_sec - GnssTimestampSec(before) > max_gap_sec ||
      GnssTimestampSec(after) - timestamp_sec > max_gap_sec) {
    return false;
  }
  *pose = InterpolatePose(before, after, timestamp_sec);
  return true;
}

bool IsRtkFixedGnss(const LocalizationEstimation& gnss) {
  return gnss.has_localization_status() &&
         gnss.localization_status() == LocalizationEstimation::LANE;
}

bool InterpolateRtkFixedGnssPose(
    const std::vector<LocalizationEstimation>& gnss_msgs, double timestamp_sec,
    double max_gap_sec, gtsam::Pose3* pose) {
  if (gnss_msgs.empty()) {
    return false;
  }

  auto iter =
      std::lower_bound(gnss_msgs.begin(), gnss_msgs.end(), timestamp_sec,
                       [](const LocalizationEstimation& gnss, double ts) {
                         return GnssTimestampSec(gnss) < ts;
                       });

  if (iter == gnss_msgs.begin()) {
    const double diff = std::abs(GnssTimestampSec(*iter) - timestamp_sec);
    if (diff > max_gap_sec || !IsRtkFixedGnss(*iter)) {
      return false;
    }
    *pose = GnssToPose(*iter);
    return true;
  }

  if (iter == gnss_msgs.end()) {
    const auto& last = gnss_msgs.back();
    const double diff = std::abs(timestamp_sec - GnssTimestampSec(last));
    if (diff > max_gap_sec || !IsRtkFixedGnss(last)) {
      return false;
    }
    *pose = GnssToPose(last);
    return true;
  }

  const auto& after = *iter;
  const auto& before = *std::prev(iter);
  if (timestamp_sec - GnssTimestampSec(before) > max_gap_sec ||
      GnssTimestampSec(after) - timestamp_sec > max_gap_sec ||
      !IsRtkFixedGnss(before) || !IsRtkFixedGnss(after)) {
    return false;
  }
  *pose = InterpolatePose(before, after, timestamp_sec);
  return true;
}

double WrapAngle(double angle) {
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

double YawOfPose(const gtsam::Pose3& pose) { return pose.rotation().yaw(); }

Eigen::Vector3d RpyOfPose(const gtsam::Pose3& pose) {
  const gtsam::Rot3& r = pose.rotation();
  return Eigen::Vector3d(r.roll(), r.pitch(), r.yaw());
}

double PositionError(const gtsam::Pose3& pose, const gtsam::Pose3& reference) {
  return (pose.translation() - reference.translation()).norm();
}

double RotationErrorRad(const gtsam::Rot3& rotation) {
  const Eigen::Quaterniond q = rotation.toQuaternion().normalized();
  return Eigen::AngleAxisd(q).angle();
}

RpeMetrics ComputeRpe(const gtsam::Pose3& reference_from,
                      const gtsam::Pose3& reference_to,
                      const gtsam::Pose3& evaluated_from,
                      const gtsam::Pose3& evaluated_to) {
  const gtsam::Pose3 reference_relative = reference_from.between(reference_to);
  const gtsam::Pose3 evaluated_relative = evaluated_from.between(evaluated_to);
  const gtsam::Pose3 error = reference_relative.inverse() * evaluated_relative;

  RpeMetrics metrics;
  metrics.valid = true;
  metrics.translation_error_m = error.translation().norm();
  metrics.rotation_error_rad = RotationErrorRad(error.rotation());
  metrics.yaw_error_rad =
      WrapAngle(YawOfPose(evaluated_relative) - YawOfPose(reference_relative));
  return metrics;
}

RpeMetrics ComputeOptimizedRpe(const std::vector<OptimizerNode>& nodes,
                               const gtsam::Values& result, std::size_t from,
                               std::size_t to) {
  if (from >= to || to >= nodes.size()) {
    return RpeMetrics();
  }
  return ComputeRpe(nodes[from].aligned_lio_pose, nodes[to].aligned_lio_pose,
                    result.at<gtsam::Pose3>(X(from)),
                    result.at<gtsam::Pose3>(X(to)));
}

RpeMetrics ComputeOptimizedWindowRpe(const std::vector<OptimizerNode>& nodes,
                                     const gtsam::Values& result,
                                     std::size_t to, double window_sec) {
  if (to == 0) {
    return RpeMetrics();
  }
  const double target_ts = nodes[to].timestamp_sec - window_sec;
  if (target_ts < nodes.front().timestamp_sec) {
    return RpeMetrics();
  }
  auto iter = std::lower_bound(
      nodes.begin(), nodes.begin() + static_cast<std::ptrdiff_t>(to), target_ts,
      [](const OptimizerNode& node, double ts) {
        return node.timestamp_sec < ts;
      });
  if (iter == nodes.begin() + static_cast<std::ptrdiff_t>(to)) {
    return RpeMetrics();
  }
  if (iter != nodes.begin()) {
    const auto prev = std::prev(iter);
    if (std::abs(prev->timestamp_sec - target_ts) <
        std::abs(iter->timestamp_sec - target_ts)) {
      iter = prev;
    }
  }
  return ComputeOptimizedRpe(
      nodes, result, static_cast<std::size_t>(iter - nodes.begin()), to);
}

void WriteRpeFields(std::ostream& os, const RpeMetrics& metrics) {
  os << metrics.valid << "," << metrics.translation_error_m << ","
     << metrics.rotation_error_rad << "," << metrics.yaw_error_rad;
}

bool EnsureParentDirectory(const std::string& path);

void WritePoseFields(std::ostream& os, const gtsam::Pose3& pose) {
  const gtsam::Point3 t = pose.translation();
  const Eigen::Quaterniond q = pose.rotation().toQuaternion().normalized();
  const Eigen::Vector3d rpy = RpyOfPose(pose);
  os << t.x() << "," << t.y() << "," << t.z() << "," << q.x() << "," << q.y()
     << "," << q.z() << "," << q.w() << "," << rpy.x() << "," << rpy.y() << ","
     << rpy.z();
}

bool WriteEvaluationCsv(const std::string& path,
                        const std::vector<OptimizerNode>& nodes,
                        const gtsam::Values& result) {
  if (path.empty()) {
    return true;
  }
  if (!EnsureParentDirectory(path)) {
    return false;
  }
  std::ofstream ofs(path);
  if (!ofs) {
    LOG(ERROR) << "failed to open output eval csv: " << path;
    return false;
  }

  ofs << "timestamp_sec,"
      << "gnss_x,gnss_y,gnss_z,gnss_qx,gnss_qy,gnss_qz,gnss_qw,gnss_roll,gnss_"
         "pitch,"
         "gnss_yaw,"
      << "aligned_lio_x,aligned_lio_y,aligned_lio_z,aligned_lio_qx,aligned_lio_"
         "qy,"
         "aligned_lio_qz,aligned_lio_qw,aligned_lio_roll,aligned_lio_pitch,"
         "aligned_lio_yaw,"
      << "optimized_x,optimized_y,optimized_z,optimized_qx,optimized_qy,"
         "optimized_qz,"
         "optimized_qw,optimized_roll,optimized_pitch,optimized_yaw,"
      << "aligned_lio_pos_error_m,optimized_pos_error_m,aligned_lio_yaw_error_"
         "rad,"
         "optimized_yaw_error_rad,"
      << "aligned_lio_error_x_m,aligned_lio_error_y_m,aligned_lio_error_z_m,"
         "aligned_lio_error_roll_rad,aligned_lio_error_pitch_rad,aligned_lio_"
         "error_yaw_rad,"
      << "optimized_error_x_m,optimized_error_y_m,optimized_error_z_m,"
         "optimized_error_roll_rad,optimized_error_pitch_rad,optimized_error_"
         "yaw_rad,"
      << "has_gnss_factor,optimized_delta_from_aligned_lio_m,"
         "optimized_delta_from_aligned_lio_yaw_rad,"
         "rpe_adjacent_valid,rpe_adjacent_trans_m,rpe_adjacent_rot_rad,"
         "rpe_adjacent_yaw_rad,"
         "rpe_1s_valid,rpe_1s_trans_m,rpe_1s_rot_rad,rpe_1s_yaw_rad,"
         "rpe_3s_valid,rpe_3s_trans_m,rpe_3s_rot_rad,rpe_3s_yaw_rad,"
         "rpe_5s_valid,rpe_5s_trans_m,rpe_5s_rot_rad,rpe_5s_yaw_rad\n";

  ofs << std::fixed << std::setprecision(9);
  for (std::size_t i = 0; i < nodes.size(); ++i) {
    const gtsam::Pose3 optimized = result.at<gtsam::Pose3>(X(i));
    const gtsam::Pose3& gnss = nodes[i].gnss_pose;
    const gtsam::Pose3& aligned_lio = nodes[i].aligned_lio_pose;
    const gtsam::Point3 gnss_t = gnss.translation();
    const gtsam::Point3 aligned_t = aligned_lio.translation();
    const gtsam::Point3 opt_t = optimized.translation();
    const Eigen::Vector3d gnss_rpy = RpyOfPose(gnss);
    const Eigen::Vector3d aligned_rpy = RpyOfPose(aligned_lio);
    const Eigen::Vector3d opt_rpy = RpyOfPose(optimized);
    const RpeMetrics adjacent_rpe =
        i > 0 ? ComputeOptimizedRpe(nodes, result, i - 1, i) : RpeMetrics();
    const RpeMetrics rpe_1s = ComputeOptimizedWindowRpe(nodes, result, i, 1.0);
    const RpeMetrics rpe_3s = ComputeOptimizedWindowRpe(nodes, result, i, 3.0);
    const RpeMetrics rpe_5s = ComputeOptimizedWindowRpe(nodes, result, i, 5.0);

    ofs << nodes[i].timestamp_sec << ",";
    WritePoseFields(ofs, gnss);
    ofs << ",";
    WritePoseFields(ofs, aligned_lio);
    ofs << ",";
    WritePoseFields(ofs, optimized);
    ofs << "," << PositionError(aligned_lio, gnss) << ","
        << PositionError(optimized, gnss) << ","
        << WrapAngle(YawOfPose(aligned_lio) - YawOfPose(gnss)) << ","
        << WrapAngle(YawOfPose(optimized) - YawOfPose(gnss)) << ","
        << aligned_t.x() - gnss_t.x() << "," << aligned_t.y() - gnss_t.y()
        << "," << aligned_t.z() - gnss_t.z() << ","
        << WrapAngle(aligned_rpy.x() - gnss_rpy.x()) << ","
        << WrapAngle(aligned_rpy.y() - gnss_rpy.y()) << ","
        << WrapAngle(aligned_rpy.z() - gnss_rpy.z()) << ","
        << opt_t.x() - gnss_t.x() << "," << opt_t.y() - gnss_t.y() << ","
        << opt_t.z() - gnss_t.z() << ","
        << WrapAngle(opt_rpy.x() - gnss_rpy.x()) << ","
        << WrapAngle(opt_rpy.y() - gnss_rpy.y()) << ","
        << WrapAngle(opt_rpy.z() - gnss_rpy.z()) << ","
        << nodes[i].has_gnss_factor << ","
        << PositionError(optimized, aligned_lio) << ","
        << WrapAngle(YawOfPose(optimized) - YawOfPose(aligned_lio)) << ",";
    WriteRpeFields(ofs, adjacent_rpe);
    ofs << ",";
    WriteRpeFields(ofs, rpe_1s);
    ofs << ",";
    WriteRpeFields(ofs, rpe_3s);
    ofs << ",";
    WriteRpeFields(ofs, rpe_5s);
    ofs << "\n";
  }
  return ofs.good();
}

bool WriteGraphNodesCsv(const std::string& path,
                        const std::vector<OptimizerNode>& nodes,
                        const gtsam::Values& result) {
  if (path.empty()) {
    return true;
  }
  if (!EnsureParentDirectory(path)) {
    return false;
  }
  std::ofstream ofs(path);
  if (!ofs) {
    LOG(ERROR) << "failed to open output graph nodes csv: " << path;
    return false;
  }

  ofs << "index,timestamp_sec,"
      << "aligned_lio_x,aligned_lio_y,aligned_lio_z,"
      << "optimized_x,optimized_y,optimized_z,"
      << "gnss_x,gnss_y,gnss_z,"
      << "has_gnss_factor\n";
  ofs << std::fixed << std::setprecision(9);
  for (std::size_t i = 0; i < nodes.size(); ++i) {
    const gtsam::Point3 aligned = nodes[i].aligned_lio_pose.translation();
    const gtsam::Point3 optimized = result.at<gtsam::Pose3>(X(i)).translation();
    const gtsam::Point3 gnss = nodes[i].gnss_pose.translation();
    ofs << i << "," << nodes[i].timestamp_sec << "," << aligned.x() << ","
        << aligned.y() << "," << aligned.z() << "," << optimized.x() << ","
        << optimized.y() << "," << optimized.z() << "," << gnss.x() << ","
        << gnss.y() << "," << gnss.z() << "," << nodes[i].has_gnss_factor
        << "\n";
  }
  return ofs.good();
}

bool WriteGraphFactorsCsv(const std::string& path,
                          const std::vector<GraphFactorRecord>& factors) {
  if (path.empty()) {
    return true;
  }
  if (!EnsureParentDirectory(path)) {
    return false;
  }
  std::ofstream ofs(path);
  if (!ofs) {
    LOG(ERROR) << "failed to open output graph factors csv: " << path;
    return false;
  }

  ofs << "type,from,to\n";
  for (const GraphFactorRecord& factor : factors) {
    ofs << factor.type << "," << factor.from << "," << factor.to << "\n";
  }
  return ofs.good();
}

bool EnsureDirectory(const std::string& dir) {
  if (dir.empty()) {
    return true;
  }

  std::string current;
  if (dir.front() == '/') {
    current = "/";
  }

  std::size_t start = dir.front() == '/' ? 1 : 0;
  while (start <= dir.size()) {
    const std::size_t end = dir.find('/', start);
    const std::string part = dir.substr(
        start, end == std::string::npos ? std::string::npos : end - start);
    if (!part.empty()) {
      if (current.size() > 1 && current.back() != '/') {
        current += '/';
      }
      current += part;
      if (mkdir(current.c_str(), 0755) != 0 && errno != EEXIST) {
        LOG(ERROR) << "failed to create directory " << current << ": "
                   << strerror(errno);
        return false;
      }
    }
    if (end == std::string::npos) {
      break;
    }
    start = end + 1;
  }
  return true;
}

bool EnsureParentDirectory(const std::string& path) {
  const std::size_t slash = path.find_last_of('/');
  if (slash == std::string::npos) {
    return true;
  }
  return EnsureDirectory(path.substr(0, slash));
}

class StateEstimator {
 public:
  bool Run() {
    if (!LoadInputs()) {
      return false;
    }
    if (!BuildNodes()) {
      return false;
    }
    gtsam::Values result;
    std::vector<GraphFactorRecord> graph_factors;
    if (!Optimize(&result, &graph_factors)) {
      return false;
    }
    return SaveOutputs(result, graph_factors);
  }

 private:
  bool LoadInputs() {
    frames_ = ReadMetaFile<Frame>(FLAGS_lidar_metadata);
    gnss_msgs_ = ReadMetaFile<LocalizationEstimation>(FLAGS_localization_bin);
    if (frames_.empty()) {
      LOG(ERROR) << "no frames loaded from " << FLAGS_lidar_metadata;
      return false;
    }
    if (gnss_msgs_.empty()) {
      LOG(ERROR) << "no localization messages loaded from "
                 << FLAGS_localization_bin;
      return false;
    }
    std::sort(gnss_msgs_.begin(), gnss_msgs_.end(),
              [](const auto& a, const auto& b) {
                return GnssTimestampSec(a) < GnssTimestampSec(b);
              });
    LOG(INFO) << "loaded frames=" << frames_.size()
              << ", gnss=" << gnss_msgs_.size();
    return true;
  }

  bool BuildNodes() {
    nodes_.clear();
    std::size_t skipped_without_lio = 0;
    std::size_t skipped_without_gnss = 0;
    std::size_t skipped_non_fixed_gnss_factor = 0;
    std::size_t skipped_gnss_factor_by_distance = 0;
    double accumulated_lio_distance_m = 0.0;
    double last_gnss_factor_distance_m =
        -std::numeric_limits<double>::infinity();
    bool has_previous_lio_pose = false;
    gtsam::Pose3 previous_lio_pose;
    for (std::size_t i = 0; i < frames_.size(); ++i) {
      const Frame& frame = frames_[i];
      if (!frame.has_timestamp_ns() || !frame.has_lio_pose_3d()) {
        ++skipped_without_lio;
        continue;
      }
      const double ts = FrameTimestampSec(frame);
      gtsam::Pose3 gnss_pose;
      gtsam::Pose3 fixed_gnss_pose;
      if (!InterpolateGnssPose(gnss_msgs_, ts, FLAGS_max_gnss_gap_sec,
                               &gnss_pose)) {
        ++skipped_without_gnss;
        continue;
      }
      OptimizerNode node;
      node.frame_index = i;
      node.timestamp_sec = ts;
      node.lio_pose = ToGtsamPose(frame.lio_pose_3d());
      if (has_previous_lio_pose) {
        const gtsam::Point3 prev_t = previous_lio_pose.translation();
        const gtsam::Point3 curr_t = node.lio_pose.translation();
        accumulated_lio_distance_m +=
            std::hypot(curr_t.x() - prev_t.x(), curr_t.y() - prev_t.y());
      }
      has_previous_lio_pose = true;
      previous_lio_pose = node.lio_pose;
      node.gnss_pose = gnss_pose;
      node.has_gnss_factor = InterpolateRtkFixedGnssPose(
          gnss_msgs_, ts, FLAGS_max_gnss_gap_sec, &fixed_gnss_pose);
      if (node.has_gnss_factor) {
        if (FLAGS_min_gnss_factor_distance_m > 0.0 &&
            accumulated_lio_distance_m - last_gnss_factor_distance_m <
                FLAGS_min_gnss_factor_distance_m) {
          node.has_gnss_factor = false;
          ++skipped_gnss_factor_by_distance;
        } else {
          node.gnss_pose = fixed_gnss_pose;
          last_gnss_factor_distance_m = accumulated_lio_distance_m;
        }
      } else {
        ++skipped_non_fixed_gnss_factor;
      }
      nodes_.push_back(node);
    }
    LOG(INFO) << "optimizer nodes=" << nodes_.size()
              << ", skipped_without_lio=" << skipped_without_lio
              << ", skipped_without_gnss=" << skipped_without_gnss
              << ", skipped_non_fixed_gnss_factor="
              << skipped_non_fixed_gnss_factor
              << ", skipped_gnss_factor_by_distance="
              << skipped_gnss_factor_by_distance
              << ", min_gnss_factor_distance_m="
              << FLAGS_min_gnss_factor_distance_m;
    if (nodes_.size() < 2) {
      LOG(ERROR) << "need at least 2 nodes";
      return false;
    }
    auto anchor_iter =
        std::find_if(nodes_.begin(), nodes_.end(),
                     [](const auto& node) { return node.has_gnss_factor; });
    if (anchor_iter == nodes_.end()) {
      LOG(ERROR) << "no RTK fixed GNSS poses found";
      return false;
    }
    const gtsam::Pose3 T_global_lio =
        anchor_iter->gnss_pose * anchor_iter->lio_pose.inverse();
    local_to_global_ =
        FLAGS_use_local_coordinate
            ? gtsam::Pose3(gtsam::Rot3(), anchor_iter->gnss_pose.translation())
            : gtsam::Pose3();
    const gtsam::Pose3 global_to_local = local_to_global_.inverse();
    for (auto& node : nodes_) {
      node.aligned_lio_pose = global_to_local * T_global_lio * node.lio_pose;
      node.gnss_pose = global_to_local * node.gnss_pose;
    }
    return true;
  }

  bool Optimize(gtsam::Values* result,
                std::vector<GraphFactorRecord>* graph_factors) const {
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;
    graph_factors->clear();
    std::size_t gnss_factor_count = 0;
    for (std::size_t i = 0; i < nodes_.size(); ++i) {
      if (nodes_[i].has_gnss_factor) {
        ++gnss_factor_count;
      }
    }
    if (gnss_factor_count < 2) {
      LOG(ERROR) << "need at least 2 RTK fixed GNSS factors, got "
                 << gnss_factor_count;
      return false;
    }

    const auto pose_noise = [](double rot_sigma, double trans_sigma) {
      return gtsam::noiseModel::Diagonal::Sigmas(
          (gtsam::Vector6() << rot_sigma, rot_sigma, rot_sigma, trans_sigma,
           trans_sigma, trans_sigma)
              .finished());
    };

    const auto lio_noise =
        pose_noise(FLAGS_lio_rotation_sigma_rad, FLAGS_lio_translation_sigma);
    const auto first_prior_noise =
        pose_noise(FLAGS_first_prior_rotation_sigma_rad,
                   FLAGS_first_prior_translation_sigma);
    const auto gnss_position_noise = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector3() << FLAGS_gnss_xy_sigma, FLAGS_gnss_xy_sigma,
         FLAGS_gnss_z_sigma)
            .finished());

    if (FLAGS_add_first_prior) {
      graph.addPrior(X(0), nodes_.front().aligned_lio_pose, first_prior_noise);
      graph_factors->push_back({"first_prior", 0, 0});
    }

    for (std::size_t i = 0; i < nodes_.size(); ++i) {
      initial.insert(X(i), nodes_[i].aligned_lio_pose);

      if (i > 0) {
        const gtsam::Pose3 relative =
            nodes_[i - 1].aligned_lio_pose.between(nodes_[i].aligned_lio_pose);
        graph.add(gtsam::BetweenFactor<gtsam::Pose3>(X(i - 1), X(i), relative,
                                                     lio_noise));
        graph_factors->push_back({"lio_between", i - 1, i});
      }

      if (nodes_[i].has_gnss_factor) {
        graph.add(gtsam::GPSFactor(X(i), nodes_[i].gnss_pose.translation(),
                                   gnss_position_noise));
        graph_factors->push_back({"gps", i, i});
      }
    }

    LOG(INFO) << "factor graph: dense_nodes=" << nodes_.size()
              << ", factors=" << graph.size()
              << ", rtk_fixed_gnss_factors=" << gnss_factor_count
              << ", lio_translation_sigma=" << FLAGS_lio_translation_sigma
              << ", lio_rotation_sigma_rad=" << FLAGS_lio_rotation_sigma_rad
              << ", gnss_xy_sigma=" << FLAGS_gnss_xy_sigma
              << ", gnss_z_sigma=" << FLAGS_gnss_z_sigma
              << ", min_gnss_factor_distance_m="
              << FLAGS_min_gnss_factor_distance_m
              << ", use_local_coordinate=" << FLAGS_use_local_coordinate
              << ", add_first_prior=" << FLAGS_add_first_prior
              << ", initial_error=" << graph.error(initial);

    gtsam::LevenbergMarquardtParams params;
    params.setMaxIterations(10000);
    params.setAbsoluteErrorTol(1e-5);
    params.setRelativeErrorTol(1e-6);
    params.setLinearSolverType("SEQUENTIAL_CHOLESKY");
    params.setlambdaUpperBound(1e20);
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial, params);
    *result = optimizer.optimize();
    LOG(INFO) << "final_error=" << graph.error(*result);
    return true;
  }

  bool SaveOutputs(const gtsam::Values& result,
                   const std::vector<GraphFactorRecord>& graph_factors) {
    for (std::size_t i = 0; i < nodes_.size(); ++i) {
      Frame& frame = frames_[nodes_[i].frame_index];
      FillPoseMessage(local_to_global_ * result.at<gtsam::Pose3>(X(i)),
                      frame.mutable_refined_pose_3d());
      FillPoseMessage(local_to_global_ * nodes_[i].gnss_pose,
                      frame.mutable_gnss_pose_3d());
    }

    if (!WriteMetaFile(FLAGS_lidar_metadata, frames_)) {
      LOG(ERROR) << "failed to write output meta: " << FLAGS_lidar_metadata;
      return false;
    }
    if (!WriteEvaluationCsv(FLAGS_output_eval_csv, nodes_, result)) {
      return false;
    }
    if (!WriteGraphNodesCsv(FLAGS_output_graph_nodes_csv, nodes_, result)) {
      return false;
    }
    if (!WriteGraphFactorsCsv(FLAGS_output_graph_factors_csv, graph_factors)) {
      return false;
    }
    LOG(INFO) << "saved optimized nodes=" << nodes_.size();
    return true;
  }

  std::vector<Frame> frames_;
  std::vector<LocalizationEstimation> gnss_msgs_;
  std::vector<OptimizerNode> nodes_;
  gtsam::Pose3 local_to_global_;
};

}  // namespace
}  // namespace mapping
}  // namespace adlabel

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_logtostderr = true;
  google::InitGoogleLogging(argv[0]);

  CHECK(!FLAGS_lidar_metadata.empty()) << "--lidar_metadata is required";
  CHECK(!FLAGS_localization_bin.empty()) << "--localization_bin is required";
  adlabel::mapping::StateEstimator estimator;
  return estimator.Run() ? 0 : 1;
}
