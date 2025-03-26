#pragma once
#include "convex_polygon_set.h"

namespace dairlib {
namespace geometry {

class PolygonHeightMap {
 public:
  explicit PolygonHeightMap(const ConvexPolygonSet& polygons, double resolution);

  double CalcHeight(double x, double y) const;

 private:

  int x_idx(double x) const {
    return std::clamp(
        static_cast<int>((x - xmin_) / resolution_), 0,nx_ - 1);
  }
  int y_idx(double y) const {
    return std::clamp(
        static_cast<int>((y - ymin_) / resolution_), 0, ny_ - 1);
  }

  Eigen::MatrixXf data_;

  double resolution_;
  double xmin_;
  double xmax_;
  double ymin_;
  double ymax_;
  int nx_;
  int ny_;
};

}
}
