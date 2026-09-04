#include "systems/senders/cost_colormap.h"

#include <algorithm>
#include <cmath>

#include "drake/common/drake_throw.h"

namespace dairlib {
namespace systems {

const Eigen::MatrixXi& RdYlGnReversedTable() {
  static const Eigen::MatrixXi* table = []() {
    auto* mat = new Eigen::MatrixXi(kNumColormapEntries, 3);
    *mat << 0, 104, 55, 2, 108, 57, 5, 113, 60, 7, 117, 62, 10, 123, 65, 12,
        127, 67, 15, 132, 70, 18, 138, 73, 20, 142, 75, 23, 147, 78, 25, 151,
        80, 33, 156, 82, 42, 160, 84, 48, 163, 86, 57, 167, 88, 63, 170, 89, 72,
        174, 92, 78, 177, 93, 87, 182, 95, 96, 186, 98, 102, 189, 99, 110, 192,
        100, 115, 194, 100, 122, 198, 101, 130, 201, 102, 135, 203, 103, 142,
        207, 103, 147, 209, 104, 155, 212, 105, 160, 214, 105, 167, 217, 107,
        173, 220, 111, 177, 222, 113, 183, 224, 117, 187, 226, 120, 193, 229,
        123, 199, 231, 127, 203, 233, 130, 209, 236, 134, 213, 237, 136, 218,
        240, 141, 223, 242, 147, 226, 243, 151, 230, 245, 157, 233, 246, 161,
        238, 248, 168, 241, 249, 172, 245, 251, 178, 250, 253, 184, 253, 254,
        188, 255, 253, 188, 255, 251, 184, 255, 247, 178, 255, 243, 172, 255,
        241, 168, 254, 237, 161, 254, 235, 157, 254, 231, 151, 254, 229, 147,
        254, 225, 141, 254, 220, 136, 254, 216, 132, 254, 210, 127, 254, 206,
        124, 254, 200, 119, 253, 195, 114, 253, 191, 111, 253, 185, 106, 253,
        181, 103, 253, 175, 98, 252, 168, 94, 251, 163, 92, 250, 155, 88, 250,
        150, 86, 249, 142, 82, 248, 137, 80, 247, 129, 76, 246, 122, 73, 245,
        117, 71, 244, 109, 67, 242, 104, 65, 238, 97, 62, 235, 90, 58, 233, 85,
        56, 229, 78, 53, 227, 73, 51, 224, 66, 47, 221, 61, 45, 218, 54, 42,
        214, 47, 39, 210, 43, 39, 204, 38, 39, 200, 34, 39, 194, 28, 39, 189,
        23, 38, 185, 19, 38, 179, 13, 38, 175, 9, 38, 169, 4, 38, 165, 0, 38;
    return mat;
  }();
  DRAKE_DEMAND(table->rows() == kNumColormapEntries && table->cols() == 3);
  return *table;
}

const Eigen::VectorXf& RdYlGnReversedPositions() {
  static const Eigen::VectorXf* positions = []() {
    auto* vec = new Eigen::VectorXf(kNumColormapEntries);
    for (int i = 0; i < kNumColormapEntries; ++i) {
      (*vec)(i) = static_cast<float>(i) / (kNumColormapEntries - 1);
    }
    return vec;
  }();
  return *positions;
}

Eigen::Vector3i RdYlGnReversedColor(double t) {
  // Clamp rather than wrap, so a value past either endpoint saturates to pure
  // green or pure red instead of aliasing to the opposite end.
  if (std::isnan(t)) {
    t = 0.0;
  }
  t = std::clamp(t, 0.0, 1.0);
  const int index = static_cast<int>(
      std::lround(t * (kNumColormapEntries - 1)));
  return RdYlGnReversedTable().row(index);
}

}  // namespace systems
}  // namespace dairlib
