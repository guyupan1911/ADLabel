#pragma once

namespace adlabel {
namespace mapping {

struct LosslessMapCell {
  float intensity = 0.f;  // aggregated mapped intensity [0, 255]
  float altitude = 0.f;   // running mean of z (meters)
  unsigned int count = 0;

  void AddSample(float new_altitude, float new_intensity) {
    AddSampleMean(new_altitude, new_intensity);
  }

  void AddSampleMean(float new_altitude, float new_intensity) {
    ++count;
    const float inv = 1.f / static_cast<float>(count);
    intensity += (new_intensity - intensity) * inv;
    altitude += (new_altitude - altitude) * inv;
  }

  void AddSampleMax(float new_altitude, float new_intensity) {
    const bool first_sample = count == 0;
    ++count;
    const float inv = 1.f / static_cast<float>(count);
    altitude += (new_altitude - altitude) * inv;
    if (first_sample || new_intensity > intensity) {
      intensity = new_intensity;
    }
  }

  void Reset() {
    intensity = 0.f;
    altitude = 0.f;
    count = 0;
  }

  unsigned char GetValue() const {
    return static_cast<unsigned char>(intensity);
  }
  float GetAlt() const { return altitude; }
  unsigned int GetCount() const { return count; }
};

}  // namespace mapping
}  // namespace adlabel
