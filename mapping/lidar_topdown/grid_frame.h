#pragma once

#include <Eigen/Core>

namespace adlabel {
namespace mapping {

struct GridFrame {
    unsigned int rows = 0;
    unsigned int cols = 0;
    double resolution = 0.0;                    // meters / pixel
    Eigen::Vector2d top_left_corner{0.0, 0.0};  // world (x,y) of pixel (row=0, col=0)

    // Convention: col increases with +x, row increases with -y.
    // Returns false if xy falls outside the grid.
    bool WorldToPixel(const Eigen::Vector2d& xy, unsigned int* row, unsigned int* col) const {
        if (resolution <= 0.0) return false;
        const double dc = (xy.x() - top_left_corner.x()) / resolution;
        const double dr = (top_left_corner.y() - xy.y()) / resolution;
        if (dc < 0.0 || dr < 0.0) return false;
        const auto c = static_cast<unsigned int>(dc);
        const auto r = static_cast<unsigned int>(dr);
        if (r >= rows || c >= cols) return false;
        *row = r;
        *col = c;
        return true;
    }

    Eigen::Vector2d PixelToWorld(unsigned int row, unsigned int col) const {
        return {top_left_corner.x() + (col + 0.5) * resolution,
                top_left_corner.y() - (row + 0.5) * resolution};
    }
};

}  // namespace mapping
}  // namespace adlabel
