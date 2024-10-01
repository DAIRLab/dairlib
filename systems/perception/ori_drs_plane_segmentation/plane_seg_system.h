#pragma once

#include "drake/systems/framework/leaf_system.h"
#include "geometry/convex_polygon_set.h"

#include "plane_seg/BlockFitter.hpp"

namespace dairlib::perception {

class PlaneSegSystem : public drake::systems::LeafSystem<double> {
 public:
  explicit PlaneSegSystem(std::string layer);


 private:

  std::string layer_;

  void CalcOutput(const drake::systems::Context<double>& context,
                  geometry::ConvexPolygonSet* output) const;

  planeseg::BlockFitter::Result ProcessDataAsCloud(
      const std::string& cloudFrame, planeseg::LabeledCloud::Ptr& inCloud,
      Eigen::Vector3f origin, Eigen::Vector3f lookDir) const;

};

}