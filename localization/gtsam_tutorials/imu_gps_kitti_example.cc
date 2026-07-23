#include <sys/stat.h>
#include <sys/types.h>

#include <cerrno>
#include <cmath>
#include <cstring>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <glog/logging.h>
#include <gtsam/base/Vector.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/ISAM2Params.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/dataset.h>

using namespace gtsam;
using symbol_shorthand::B;
using symbol_shorthand::V;
using symbol_shorthand::X;

struct KittiCalibration {
  double body_ptx;
  double body_pty;
  double body_ptz;
  double body_prx;
  double body_pry;
  double body_prz;

  double accelerometer_sigma;
  double gyroscope_sigma;
  double integration_sigma;
  double accelerometer_bias_sigma;
  double gyroscope_bias_sigma;
  double average_delta_t;
};

struct ImuMeasurement {
  double time;
  double dt;
  Vector3 accelerometer;
  Vector3 gyroscope;
};

struct GpsMeasurement {
  double time;
  Vector3 position;
};

namespace {

void EnsureDirectory(const std::string& dir) {
  if (dir.empty()) {
    return;
  }

  std::string current;
  if (dir.front() == '/') {
    current = "/";
  }

  size_t start = dir.front() == '/' ? 1 : 0;
  while (start <= dir.size()) {
    const size_t end = dir.find('/', start);
    const std::string part = dir.substr(
        start, end == std::string::npos ? std::string::npos : end - start);
    if (!part.empty()) {
      if (current.size() > 1 && current.back() != '/') {
        current += '/';
      }
      current += part;
      if (mkdir(current.c_str(), 0755) != 0 && errno != EEXIST) {
        LOG(FATAL) << "Failed to create directory " << current << ": "
                   << std::strerror(errno);
      }
    }
    if (end == std::string::npos) {
      break;
    }
    start = end + 1;
  }
}

void EnsureParentDirectory(const std::string& path) {
  const size_t slash = path.find_last_of('/');
  if (slash == std::string::npos) {
    return;
  }
  EnsureDirectory(path.substr(0, slash));
}

void SaveKittiTrajectoryCsv(const std::string& path,
                            const std::vector<GpsMeasurement>& gps_measurements,
                            const Values& result, size_t first_gps_pose) {
  EnsureParentDirectory(path);

  std::ofstream ofs(path);
  if (!ofs) {
    LOG(FATAL) << "Failed to open trajectory CSV for writing: " << path;
  }

  ofs << "name,key,x,y,z,theta,qx,qy,qz,qw,cov_xx,cov_xy,cov_yy,cov_tt\n";
  for (size_t i = first_gps_pose; i < gps_measurements.size() - 1; ++i) {
    const auto pose_key = X(i);
    const auto& gps_position = gps_measurements[i].position;

    ofs << "gps," << i << ',' << gps_position.x() << ',' << gps_position.y()
        << ',' << gps_position.z() << ",,,,,,,,,\n";

    if (!result.exists(pose_key)) {
      continue;
    }

    const auto pose = result.at<Pose3>(pose_key);
    const auto quat = pose.rotation().toQuaternion();
    ofs << "optimized," << i << ',' << pose.x() << ',' << pose.y() << ','
        << pose.z() << ',' << pose.rotation().yaw() << ',' << quat.x() << ','
        << quat.y() << ',' << quat.z() << ',' << quat.w() << ",,,,\n";
  }

  LOG(INFO) << "Saved trajectory CSV: " << path;
}

}  // namespace

void LoadKittiData(KittiCalibration& kitti_calibration,
                   std::vector<ImuMeasurement>& imu_measurements,
                   std::vector<GpsMeasurement>& gps_measurements) {
  std::string line;

  // read imu metadata file
  std::string imu_metadata_file =
      "localization/gtsam_tutorials/data/KittiEquivBiasedImu_metadata.txt";

  std::ifstream imu_metadata(imu_metadata_file);
  if (!imu_metadata.is_open()) {
    LOG(FATAL) << "Failed to open IMU metadata file: " << imu_metadata_file;
  }
  std::getline(imu_metadata, line, '\n');  // ignore the first line
  std::getline(imu_metadata, line, '\n');

  std::istringstream iss(line);
  if (!(iss >> kitti_calibration.body_ptx >> kitti_calibration.body_pty >>
        kitti_calibration.body_ptz >> kitti_calibration.body_prx >>
        kitti_calibration.body_pry >> kitti_calibration.body_prz >>
        kitti_calibration.accelerometer_sigma >>
        kitti_calibration.gyroscope_sigma >>
        kitti_calibration.integration_sigma >>
        kitti_calibration.accelerometer_bias_sigma >>
        kitti_calibration.gyroscope_bias_sigma >>
        kitti_calibration.average_delta_t)) {
    LOG(FATAL) << "Fail to parse imu parameters from: " << line;
  }

  LOG(INFO) << "\n"
            << "kitti_calibration.body_ptx: " << kitti_calibration.body_ptx
            << "\n"
            << "kitti_calibration.body_pty: " << kitti_calibration.body_pty
            << "\n"
            << "kitti_calibration.body_ptz: " << kitti_calibration.body_ptz
            << "\n"
            << "kitti_calibration.body_prx: " << kitti_calibration.body_prx
            << "\n"
            << "kitti_calibration.body_pry: " << kitti_calibration.body_pry
            << "\n"
            << "kitti_calibration.body_prz: " << kitti_calibration.body_prz
            << "\n"
            << "kitti_calibration.accelerometer_sigma: "
            << kitti_calibration.accelerometer_sigma << "\n"
            << "kitti_calibration.gyroscope_sigma: "
            << kitti_calibration.gyroscope_sigma << "\n"
            << "kitti_calibration.integration_sigma: "
            << kitti_calibration.integration_sigma << "\n"
            << "kitti_calibration.accelerometer_bias_sigma: "
            << kitti_calibration.accelerometer_bias_sigma << "\n"
            << "kitti_calibration.gyroscope_bias_sigma: "
            << kitti_calibration.gyroscope_bias_sigma << "\n"
            << "kitti_calibration.average_delta_t: "
            << kitti_calibration.average_delta_t;

  // read imu data
  std::string imu_measurements_file =
      "localization/gtsam_tutorials/data/KittiEquivBiasedImu.txt";
  std::ifstream imu_data(imu_measurements_file);
  if (!imu_data.is_open()) {
    LOG(FATAL) << "Failed to open IMU measurements file: "
               << imu_measurements_file;
  }
  std::getline(imu_data, line, '\n');  // ignore the first line

  while (std::getline(imu_data, line, '\n')) {
    if (line.empty()) {
      continue;
    }
    std::istringstream iss(line);
    ImuMeasurement measurement;
    if (!(iss >> measurement.time >> measurement.dt >>
          measurement.accelerometer[0] >> measurement.accelerometer[1] >>
          measurement.accelerometer[2] >> measurement.gyroscope[0] >>
          measurement.gyroscope[1] >> measurement.gyroscope[2])) {
      LOG(ERROR) << "Fail to parse imu measurement: " << line;
      continue;
    }

    imu_measurements.push_back(measurement);
  }
  LOG(INFO) << "Read " << imu_measurements.size() << " IMU measurements";

  // read gps data
  std::string gps_measurements_file =
      "localization/gtsam_tutorials/data/KittiGps_converted.txt";
  std::ifstream gps_data(gps_measurements_file);
  if (!gps_data.is_open()) {
    LOG(FATAL) << "Failed to open GPS measurements file: "
               << gps_measurements_file;
  }
  std::getline(gps_data, line, '\n');  // ignore the first line

  while (std::getline(gps_data, line, '\n')) {
    if (line.empty()) {
      continue;
    }
    std::istringstream iss(line);
    GpsMeasurement measurement;
    char comma1 = '\0';
    char comma2 = '\0';
    char comma3 = '\0';
    if (!(iss >> measurement.time >> comma1 >> measurement.position[0] >>
          comma2 >> measurement.position[1] >> comma3 >>
          measurement.position[2]) ||
        comma1 != ',' || comma2 != ',' || comma3 != ',') {
      LOG(ERROR) << "Fail to parse gps measurement: " << line;
      continue;
    }
    gps_measurements.push_back(measurement);
  }
  LOG(INFO) << "Read " << gps_measurements.size() << " GPS measurements";
}

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);

  KittiCalibration kitti_calibration;
  std::vector<ImuMeasurement> imu_measurements;
  std::vector<GpsMeasurement> gps_measurements;
  LoadKittiData(kitti_calibration, imu_measurements, gps_measurements);

  // set initial
  size_t first_gps_pose = 1;
  if (gps_measurements.size() <= first_gps_pose) {
    LOG(FATAL) << "Not enough GPS measurements: " << gps_measurements.size();
  }
  if (imu_measurements.empty()) {
    LOG(FATAL) << "No IMU measurements were loaded";
  }
  // Accumulate several GPS/IMU constraints before the first iSAM2 update so
  // heading/yaw is observable from motion.
  size_t initialization_gps_count = 20;
  double g = 9.8;
  auto w_coriolis = Vector3(0, 0, 0);

  auto noise_model_gps = noiseModel::Diagonal::Precisions(
      (Vector6() << Vector3::Constant(0), Vector3::Constant(1.0 / 0.07))
          .finished());

  auto current_pose_global =
      Pose3(Rot3(), gps_measurements[first_gps_pose].position);
  Vector3 current_velocity_global = Vector3::Zero();
  auto current_bias = imuBias::ConstantBias();

  auto sigma_init_x = noiseModel::Diagonal::Precisions(
      (Vector6() << Vector3::Constant(0), Vector3::Constant(1.0)).finished());
  auto sigma_init_v = noiseModel::Diagonal::Sigmas(Vector3::Constant(1000));
  auto sigma_init_b = noiseModel::Diagonal::Sigmas(
      (Vector6() << Vector3::Constant(0.1), Vector3::Constant(5e-5))
          .finished());

  // set imu preintegration parameters
  Matrix33 measured_acc_cov =
      I_3x3 * std::pow(kitti_calibration.accelerometer_sigma, 2);
  Matrix33 measured_omega_cov =
      I_3x3 * std::pow(kitti_calibration.gyroscope_sigma, 2);
  Matrix33 integration_error_cov =
      I_3x3 * std::pow(kitti_calibration.integration_sigma, 2);

  auto imu_params = PreintegrationParams::MakeSharedU(g);
  imu_params->accelerometerCovariance = measured_acc_cov;
  imu_params->gyroscopeCovariance = measured_omega_cov;
  imu_params->integrationCovariance = integration_error_cov;
  imu_params->omegaCoriolis = w_coriolis;

  std::shared_ptr<PreintegratedImuMeasurements> current_summarized_measurement;

  // Set ISAM2 parameters and create ISAM2 solver object
  ISAM2Params isam_params;
  isam_params.factorization = ISAM2Params::CHOLESKY;
  isam_params.relinearizeSkip = 10;

  ISAM2 isam(isam_params);

  // create factor graph and values
  NonlinearFactorGraph new_factors;
  Values new_values;

  LOG(INFO) << "Starting main loop\n";

  size_t j = 0;
  for (size_t i = first_gps_pose; i < gps_measurements.size() - 1; i++) {
    // At each gps measurement, initialize a new node in the graph
    auto current_pose_key = X(i);
    auto current_vel_key = V(i);
    auto current_bias_key = B(i);
    double t = gps_measurements[i].time;
    size_t included_imu_measurement_count = 0;

    if (i == first_gps_pose) {
      new_values.insert(current_pose_key, current_pose_global);
      new_values.insert(current_vel_key, current_velocity_global);
      new_values.insert(current_bias_key, current_bias);
      new_factors.emplace_shared<PriorFactor<Pose3>>(
          current_pose_key, current_pose_global, sigma_init_x);
      new_factors.emplace_shared<PriorFactor<Vector3>>(
          current_vel_key, current_velocity_global, sigma_init_v);
      new_factors.emplace_shared<PriorFactor<imuBias::ConstantBias>>(
          current_bias_key, current_bias, sigma_init_b);
    } else {
      double t_previous = gps_measurements[i - 1].time;
      current_summarized_measurement =
          std::make_shared<PreintegratedImuMeasurements>(imu_params,
                                                         current_bias);

      while (j < imu_measurements.size() && imu_measurements[j].time <= t) {
        if (imu_measurements[j].time >= t_previous) {
          current_summarized_measurement->integrateMeasurement(
              imu_measurements[j].accelerometer, imu_measurements[j].gyroscope,
              imu_measurements[j].dt);
          included_imu_measurement_count++;
        }
        j++;
      }

      // create imu factor
      auto previous_pose_key = X(i - 1);
      auto previous_vel_key = V(i - 1);
      auto previous_bias_key = B(i - 1);

      new_factors.emplace_shared<ImuFactor>(
          previous_pose_key, previous_vel_key, current_pose_key,
          current_vel_key, previous_bias_key, *current_summarized_measurement);

      auto sigma_between_b = noiseModel::Diagonal::Sigmas(
          (Vector6() << Vector3::Constant(
               std::sqrt(included_imu_measurement_count) *
               kitti_calibration.accelerometer_bias_sigma),
           Vector3::Constant(std::sqrt(included_imu_measurement_count) *
                             kitti_calibration.gyroscope_bias_sigma))
              .finished());

      new_factors.emplace_shared<BetweenFactor<imuBias::ConstantBias>>(
          previous_bias_key, current_bias_key, imuBias::ConstantBias(),
          sigma_between_b);

      NavState previous_state(current_pose_global, current_velocity_global);
      NavState predicted_state =
          current_summarized_measurement->predict(previous_state, current_bias);

      // create gps factor
      auto gps_pose = Pose3(predicted_state.pose().rotation(),
                            gps_measurements[i].position);
      new_factors.emplace_shared<PriorFactor<Pose3>>(current_pose_key, gps_pose,
                                                     noise_model_gps);
      Pose3 current_pose_initial = gps_pose;

      new_values.insert(current_pose_key, current_pose_initial);
      new_values.insert(current_vel_key, predicted_state.v());
      new_values.insert(current_bias_key, current_bias);
      current_pose_global = current_pose_initial;
      current_velocity_global = predicted_state.v();

      if (i > (first_gps_pose + initialization_gps_count)) {
        isam.update(new_factors, new_values);

        // Reset the newFactors and newValues list
        new_factors.resize(0);
        new_values.clear();

        // Extract the result/current estimates
        Values result = isam.calculateEstimate();

        current_pose_global = result.at<Pose3>(current_pose_key);
        current_velocity_global = result.at<Vector3>(current_vel_key);
        current_bias = result.at<imuBias::ConstantBias>(current_bias_key);
      }
    }
  }

  Values result = isam.calculateEstimate();
  const std::string trajectory_csv =
      "data/gtsam_results/imu_gps_kitti_trajectory.csv";
  SaveKittiTrajectoryCsv(trajectory_csv, gps_measurements, result,
                         first_gps_pose);

  return 0;
}
