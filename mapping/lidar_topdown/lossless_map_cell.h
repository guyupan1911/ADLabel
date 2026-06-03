#pragma once

namespace adlabel {
namespace mapping {

struct LosslessMapCell {
    float intensity = 0.f;  // running mean of intensity [0, 255]
    float altitude = 0.f;   // running mean of z (meters)
    unsigned int count = 0;

    void AddSample(float new_altitude, unsigned char new_intensity) {
        ++count;
        const float inv = 1.f / static_cast<float>(count);
        intensity += (static_cast<float>(new_intensity) - intensity) * inv;
        altitude += (new_altitude - altitude) * inv;
    }

    void Reset() {
        intensity = 0.f;
        altitude = 0.f;
        count = 0;
    }

    unsigned char GetValue() const { return static_cast<unsigned char>(intensity); }
    float GetAlt() const { return altitude; }
    unsigned int GetCount() const { return count; }
};

}  // namespace mapping
}  // namespace adlabel
