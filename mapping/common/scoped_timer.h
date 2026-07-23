#pragma once

#include <chrono>
#include <string>

namespace adlabel {
namespace mapping {

class ScopedTimer {
 public:
  explicit ScopedTimer(std::string name);
  ~ScopedTimer();

  ScopedTimer(const ScopedTimer&) = delete;
  ScopedTimer& operator=(const ScopedTimer&) = delete;

 private:
  std::string name_;
  std::chrono::steady_clock::time_point start_time_;
};

}  // namespace mapping
}  // namespace adlabel
