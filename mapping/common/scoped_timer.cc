#include "mapping/common/scoped_timer.h"

#include <chrono>
#include <utility>

#include <glog/logging.h>

namespace adlabel {
namespace mapping {

ScopedTimer::ScopedTimer(std::string name)
    : name_(std::move(name)), start_time_(std::chrono::steady_clock::now()) {}

ScopedTimer::~ScopedTimer() {
  const auto elapsed = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - start_time_);
  VLOG(1) << name_ << " took " << elapsed.count() << " ms";
}

}  // namespace mapping
}  // namespace adlabel
