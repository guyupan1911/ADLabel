#include <fstream>
#include <string>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "mapping/protos/hello_world.pb.h"

DEFINE_string(output_dir, "/tmp", "Directory to save output files");

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  FLAGS_logtostderr = true;
  google::InitGoogleLogging(argv[0]);

  const std::string& out = FLAGS_output_dir;

  // --- 创建 proto 消息 ---
  adlabel::mapping::HelloWorldData data;
  data.set_message("Hello from protobuf!");
  data.set_counter(42);

  // 填充点云帧
  auto* frame = data.mutable_frame();
  frame->set_timestamp_ns(1234567890000ULL);
  frame->set_sensor_id("lidar_front");

  for (int i = 0; i < 5; ++i) {
    auto* pt = frame->add_points();
    pt->set_x(i * 0.1);
    pt->set_y(i * 0.2);
    pt->set_z(i * 0.3);
  }

  LOG(INFO) << "Created proto message:";
  LOG(INFO) << "  message: " << data.message();
  LOG(INFO) << "  counter: " << data.counter();
  LOG(INFO) << "  frame.points: " << frame->points_size();

  // --- 序列化到文件（二进制） ---
  std::string bin_path = out + "/hello_world.pb";
  std::ofstream ofs_bin(bin_path, std::ios::binary);
  if (!data.SerializeToOstream(&ofs_bin)) {
    LOG(ERROR) << "Failed to serialize to: " << bin_path;
    return 1;
  }
  ofs_bin.close();
  LOG(INFO) << "Saved binary proto: " << bin_path << "  size="
            << std::ifstream(bin_path, std::ios::ate | std::ios::binary).tellg()
            << " bytes";

  // --- 序列化到文件（文本格式，调试用） ---
  std::string txt_path = out + "/hello_world.pb.txt";
  std::ofstream ofs_txt(txt_path);
  ofs_txt << data.DebugString();
  ofs_txt.close();
  LOG(INFO) << "Saved text proto:   " << txt_path;

  // --- 从二进制文件读取并验证 ---
  adlabel::mapping::HelloWorldData loaded;
  std::ifstream ifs_bin(bin_path, std::ios::binary);
  if (!loaded.ParseFromIstream(&ifs_bin)) {
    LOG(ERROR) << "Failed to parse from: " << bin_path;
    return 1;
  }
  ifs_bin.close();

  LOG(INFO) << "Loaded proto message:";
  LOG(INFO) << "  message: " << loaded.message();
  LOG(INFO) << "  counter: " << loaded.counter();
  LOG(INFO) << "  frame.sensor_id: " << loaded.frame().sensor_id();
  LOG(INFO) << "  frame.points: " << loaded.frame().points_size();

  // 验证数据一致性
  bool ok = (loaded.message() == data.message() &&
             loaded.counter() == data.counter() &&
             loaded.frame().points_size() == data.frame().points_size());

  if (ok) {
    LOG(INFO) << "✓ Serialization/deserialization test PASSED";
  } else {
    LOG(ERROR) << "✗ Data mismatch!";
    return 1;
  }

  return 0;
}
