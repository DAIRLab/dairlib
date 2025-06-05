#pragma once

#include <utility>
#include <vector>

#include "convex_polygon.h"
#include "dairlib/lcmt_foothold_set.hpp"

namespace dairlib{
namespace geometry{
class ConvexPolygonSet {
 public:
  ConvexPolygonSet() = default;
  explicit ConvexPolygonSet(
      std::vector<ConvexPolygon> set) : set_(std::move(set)) {};

  /*!
   * Get the subset of footholds which contain a point closer than "threshold"
   * to the query point. Constructs and solves a quadratic program with 3
   * variables per polygon in this set.
   */
  [[nodiscard]] ConvexPolygonSet GetSubsetCloseToPoint(
      const Eigen::Vector3d &query_pt, double threshold) const;

  /*!
   * @return a const reference to the underlying vector of convex polygons
   */
  const std::vector<ConvexPolygon>& polygons() const { return set_; }
  void clear() { set_.clear(); }
  bool empty() { return set_.empty(); }

  /*!
   * ReExpress all of the convex polygons in this set in a new frame. See the
   * corresponding method for ConvexPolygon.
   */
  void ReExpressInNewFrame(const Eigen::Matrix3d& R_WF);

  /*!
   * Append the convex polygon p to this set
   * @param p the polygon to apend
   */
  void append(const ConvexPolygon& p) { set_.push_back(p); }

  void CopyToLcm(lcmt_foothold_set* set) const;

  static ConvexPolygonSet CopyFromLcm(const lcmt_foothold_set& set);
  size_t size() const { return set_.size(); }

  static ConvexPolygonSet MakeFlatGround(double half_len=100.0) {
    return ConvexPolygonSet({ConvexPolygon::MakeFlatGround(half_len)});
  }

 private:
  std::vector<ConvexPolygon> set_;
};
}
}



