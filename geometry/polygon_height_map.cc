#include "polygon_height_map.h"


namespace dairlib {
namespace geometry {

static constexpr kInf = std::numeric_limits<double>::infinity();

PolygonHeightMap::PolygonHeightMap(const ConvexPolygonSet& polys, double resolution) {
  xmin_ = kInf;
  xmax_ = -kInf;
  ymin_ = kInf;
  ymax_ = -kInf;
  resolution_ = resolution;

  for (const auto& poly: polys) {
    xmin_ = poly.xmin() < xmin_ ? poly.xmin() : xmin_;
    xmin_ = poly.xmax() > xmax_ ? poly.xmax() : xmax_;
    ymin_ = poly.ymin() < ymin_ ? poly.ymin() : ymin_;
    ymin_ = poly.ymax() > ymax_ ? poly.ymax() : ymax_;
  }
  nx_ = static_cast<int>((xmax_ - xmin_) / resolution + 1.0);
  ny_ = static_cast<int>(ymax_ - ymin_ + 1.0);

  data_ = Eigen::MatrixXf::Constant(nx_, ny_, -kInf);

  for (const auto& poly: polys) {

  }

}

}
}