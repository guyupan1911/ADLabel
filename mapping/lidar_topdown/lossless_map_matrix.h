#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <unordered_map>
#include <vector>

#include "mapping/lidar_topdown/lossless_map_cell.h"

namespace adlabel {
namespace mapping {

class LosslessMapMatrix {
  public:
    virtual ~LosslessMapMatrix() = default;

    virtual void Init(unsigned int rows, unsigned int cols) = 0;
    virtual void Reset() = 0;

    virtual const LosslessMapCell* Find(unsigned int row, unsigned int col) const = 0;
    virtual LosslessMapCell& GetOrCreate(unsigned int row, unsigned int col) = 0;

    using Visitor = std::function<void(unsigned int row, unsigned int col, const LosslessMapCell&)>;
    virtual void ForEachOccupied(const Visitor& visitor) const = 0;

    unsigned int rows() const { return rows_; }
    unsigned int cols() const { return cols_; }

  protected:
    unsigned int rows_ = 0;
    unsigned int cols_ = 0;
};

// Dense backend: flat vector, O(rows*cols) memory regardless of occupancy.
class DenseLosslessMapMatrix : public LosslessMapMatrix {
  public:
    void Init(unsigned int rows, unsigned int cols) override {
        rows_ = rows;
        cols_ = cols;
        cells_.assign(static_cast<size_t>(rows) * cols, LosslessMapCell{});
    }

    void Reset() override {
        for (auto& c : cells_) c.Reset();
    }

    const LosslessMapCell* Find(unsigned int row, unsigned int col) const override {
        if (row >= rows_ || col >= cols_) return nullptr;
        const auto& c = cells_[row * cols_ + col];
        return c.count > 0 ? &c : nullptr;
    }

    LosslessMapCell& GetOrCreate(unsigned int row, unsigned int col) override {
        return cells_[row * cols_ + col];
    }

    void ForEachOccupied(const Visitor& visitor) const override {
        for (unsigned int r = 0; r < rows_; ++r) {
            for (unsigned int c = 0; c < cols_; ++c) {
                const auto& cell = cells_[r * cols_ + c];
                if (cell.count > 0) visitor(r, c, cell);
            }
        }
    }

  private:
    std::vector<LosslessMapCell> cells_;
};

// Sparse backend: hash map, only allocates occupied cells.
class SparseLosslessMapMatrix : public LosslessMapMatrix {
  public:
    void Init(unsigned int rows, unsigned int cols) override {
        rows_ = rows;
        cols_ = cols;
        cells_.clear();
    }

    void Reset() override { cells_.clear(); }

    const LosslessMapCell* Find(unsigned int row, unsigned int col) const override {
        auto it = cells_.find(Key(row, col));
        return it != cells_.end() ? &it->second : nullptr;
    }

    LosslessMapCell& GetOrCreate(unsigned int row, unsigned int col) override {
        return cells_[Key(row, col)];
    }

    void ForEachOccupied(const Visitor& visitor) const override {
        for (const auto& kv : cells_) {
            const unsigned int r = static_cast<unsigned int>(kv.first >> 32);
            const unsigned int c = static_cast<unsigned int>(kv.first & 0xFFFFFFFFu);
            visitor(r, c, kv.second);
        }
    }

  private:
    static uint64_t Key(unsigned int row, unsigned int col) {
        return (static_cast<uint64_t>(row) << 32) | col;
    }

    std::unordered_map<uint64_t, LosslessMapCell> cells_;
};

}  // namespace mapping
}  // namespace adlabel
