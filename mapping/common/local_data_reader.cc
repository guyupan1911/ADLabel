#include "mapping/common/local_data_reader.h"

#include <glog/logging.h>
#include <pcl/io/pcd_io.h>

namespace adlabel {
namespace mapping {

LocalDataReader::LocalDataReader(const std::string& data_root) {
    data_root_ = std::filesystem::path(data_root);
}

bool LocalDataReader::ReadPointCloud(const std::string& relative_path,
                                     PointCloudXYZIRT::Ptr cloud) const {
    CHECK(cloud != nullptr);

    const auto path = data_root_ / relative_path;
    if (!std::filesystem::exists(path)) {
        LOG(ERROR) << "Point cloud file does not exist: " << path.string();
        return false;
    }

    if (pcl::io::loadPCDFile<PointXYZIRT>(path.string(), *cloud) != 0) {
        LOG(ERROR) << "Failed to load point cloud: " << path.string();
        return false;
    }

    return true;
}

bool LocalDataReader::ReadImage(const std::string& relative_path, cv::Mat* image, int flags) const {
    CHECK(image != nullptr);

    const auto path = data_root_ / relative_path;
    if (!std::filesystem::exists(path)) {
        LOG(ERROR) << "Image file does not exist: " << path.string();
        return false;
    }

    *image = cv::imread(path.string(), flags);
    if (image->empty()) {
        LOG(ERROR) << "Failed to load image: " << path.string();
        return false;
    }

    return true;
}

}  // namespace mapping
}  // namespace adlabel
