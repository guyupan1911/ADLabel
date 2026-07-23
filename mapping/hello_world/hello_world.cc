#include <fstream>
#include <string>

#include <Eigen/Dense>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

DEFINE_string(output_dir, "/tmp", "Directory to save output files");

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  const std::string& out = FLAGS_output_dir;

  // --- OpenCV: create a 480x640 BGR image and save as PNG ---
  cv::Mat image(480, 640, CV_8UC3, cv::Scalar(0, 128, 255));
  cv::putText(image, "ADLabel", cv::Point(200, 240), cv::FONT_HERSHEY_SIMPLEX,
              2.0, cv::Scalar(255, 255, 255), 3);

  std::string img_path = out + "/hello_world.png";
  if (!cv::imwrite(img_path, image)) {
    LOG(ERROR) << "Failed to write image: " << img_path;
    return 1;
  }
  LOG(INFO) << "Saved image: " << img_path << "  size=" << image.cols << "x"
            << image.rows;

  // --- PCL: create a simple XYZ point cloud and save as PCD ---
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->width = 5;
  cloud->height = 1;
  cloud->is_dense = true;
  cloud->points.resize(cloud->width);
  for (size_t i = 0; i < cloud->points.size(); ++i) {
    cloud->points[i].x = static_cast<float>(i) * 0.1f;
    cloud->points[i].y = static_cast<float>(i) * 0.2f;
    cloud->points[i].z = static_cast<float>(i) * 0.3f;
  }

  std::string pcd_path = out + "/hello_world.pcd";
  if (pcl::io::savePCDFileASCII(pcd_path, *cloud) < 0) {
    LOG(ERROR) << "Failed to write PCD: " << pcd_path;
    return 1;
  }
  LOG(INFO) << "Saved PCD:   " << pcd_path
            << "  points=" << cloud->points.size();

  // --- Eigen: 构造旋转矩阵 + 平移向量，做一次刚体变换并保存 ---
  Eigen::Matrix3d R =
      Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  Eigen::Vector3d t(1.0, 2.0, 3.0);

  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T.block<3, 3>(0, 0) = R;
  T.block<3, 1>(0, 3) = t;

  Eigen::Vector4d p(1.0, 0.0, 0.0, 1.0);
  Eigen::Vector4d p_transformed = T * p;

  std::string eigen_path = out + "/hello_world_eigen.txt";
  LOG(INFO) << "eigen path: " << eigen_path;
  std::ofstream ofs(eigen_path);
  if (!ofs) {
    LOG(ERROR) << "Failed to write: " << eigen_path;
    return 1;
  }
  ofs << "# Rotation matrix (45 deg around Z)\n" << R << "\n\n";
  ofs << "# Translation vector\n" << t.transpose() << "\n\n";
  ofs << "# Transform matrix (4x4)\n" << T << "\n\n";
  ofs << "# Input point:       " << p.transpose() << "\n";
  ofs << "# Transformed point: " << p_transformed.transpose() << "\n";
  ofs.close();

  LOG(ERROR) << "Saved Eigen: " << eigen_path;
  LOG(WARNING) << "  input    = " << p.transpose();
  LOG(INFO) << "  output   = " << p_transformed.transpose();

  return 0;
}
