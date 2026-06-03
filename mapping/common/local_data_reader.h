#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include <glog/logging.h>
#include <opencv2/opencv.hpp>

#include "mapping/common/file.h"
#include "mapping/common/pcl_types.h"

namespace adlabel {
namespace mapping {

class LocalDataReader {
  public:
    explicit LocalDataReader(const std::string& data_root);

    bool ReadPointCloud(const std::string& relative_path,
                        PointCloudXYZIRT::Ptr cloud) const;

    bool ReadImage(const std::string& relative_path,
                   cv::Mat* image,
                   int flags = cv::IMREAD_UNCHANGED) const;
    
    template <typename MessageT>
    std::vector<MessageT> ReadMetaData(const std::string& relative_path) const {
      const std::filesystem::path root_path = data_root_ / relative_path;
      CHECK(std::filesystem::exists(root_path)) << root_path.string()
        << " does not exist";
      
      return ReadMetaFile<MessageT>(root_path.string());
    }

  private:
    std::filesystem::path data_root_;
};

}  // namespace mapping
}  // namespace adlabel
