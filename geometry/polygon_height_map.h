#pragma once
#include "convex_polygon_set.h"

namespace dairlib {
namespace perception {

class PolygonHeightMap {
 public:
  explicit PolygonHeightMap(const ConvexPolygonSet& polygons, double resolution);

  double CalcHeight(double x, double y);

 private:

  int x_ixd(double x) { return static_cast<int>((x - xmin_) / resolution) };
  int y_idx(double y) { return static_cast<int>((y - ymin_) / resolution) };

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
