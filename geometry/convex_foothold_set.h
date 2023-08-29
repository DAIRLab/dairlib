#pragma once

#include <utility>
#include <vector>

#include "convex_foothold.h"
#include "dairlib/lcmt_foothold_set.hpp"

namespace dairlib{
namespace geometry{
class ConvexFootholdSet {
 public:
  ConvexFootholdSet() = default;
  ConvexFootholdSet(std::vector<ConvexFoothold> set) : set_(std::move(set)) {};

  /*
   * Get the subset of footholds which contain a point closer than "threshold"
   * to the query point
   */
  ConvexFootholdSet GetSubsetCloseToPoint(
      const Eigen::Vector3d &query_pt, double threshold) const;

  ConvexFootholdSet GetSubsetInForwardLookingCone(
      const Eigen::Vector3d& query_pt, double cone_angle) const;

  const std::vector<ConvexFoothold>& footholds() const { return set_; }
  void clear() { set_.clear(); }
  bool empty() {return set_.empty(); }
  bool Feasible2d(const Eigen::Vector3d& pt, double tol = -1e-2) const;
  void ReExpressInNewFrame(const Eigen::Matrix3d& R_WF);
  void append(const ConvexFoothold& f) { set_.push_back(f); }
  void CopyToLcm(lcmt_foothold_set* set) const;
  static ConvexFootholdSet CopyFromLcm(const lcmt_foothold_set& set);
  size_t size() const { return set_.size(); }

 private:
  std::vector<ConvexFoothold> set_;
};
}
}



