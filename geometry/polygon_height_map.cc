#include "polygon_height_map.h"
#include <iostream>

namespace dairlib {
namespace geometry {

static double constexpr kInf = std::numeric_limits<double>::infinity();

PolygonHeightMap::PolygonHeightMap(const ConvexPolygonSet& polys, double resolution) {
  xmin_ = kInf;
  xmax_ = -kInf;
  ymin_ = kInf;
  ymax_ = -kInf;
  resolution_ = resolution;

  for (const auto& poly: polys.polygons()) {
    xmin_ = poly.xmin() < xmin_ ? poly.xmin() : xmin_;
    xmax_ = poly.xmax() > xmax_ ? poly.xmax() : xmax_;
    ymin_ = poly.ymin() < ymin_ ? poly.ymin() : ymin_;
    ymax_ = poly.ymax() > ymax_ ? poly.ymax() : ymax_;
  }
  nx_ = static_cast<int>((xmax_ - xmin_) / resolution + 1.0);
  ny_ = static_cast<int>(ymax_ - ymin_ + 1.0);

  data_ = Eigen::MatrixXf::Constant(nx_, ny_, -kInf);

  for (const auto& poly: polys.polygons()) {
    int idx_x_min = x_idx(poly.xmin());
    int idx_y_min = y_idx(poly.ymin());
    int idx_x_max = x_idx(poly.xmax());
    int idx_y_max = y_idx(poly.ymax());
    for (int x = idx_x_min; x <= idx_x_max; ++x) {
      for (int y = idx_y_min; y <= idx_y_max; ++y) {
        double xd = xmin_ + resolution_ * x;
        double yd = ymin_ + resolution_ * y;
        Eigen::Vector3d point(xd, yd, 0);
        if (not poly.PointViolatesInequalities(point)) {
          const auto& [A, b] = poly.GetEqualityConstraintMatrices();
          double z = (b - A.leftCols<2>() * point.head<2>())(0);
          if (z >= data_(x, y)) {
            data_(x, y) = z;
          }
        }
      }
    }
  }
}

double PolygonHeightMap::CalcHeight(double x, double y) const {
  return data_(x_idx(x), y_idx(y));
}

}
}