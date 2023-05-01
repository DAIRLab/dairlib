// STL includes
#include <iostream>

// dair includes
#include "poly_utils.h"
#include "convex_foothold_ros_receiver.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::RowVector3d;
using drake::systems::Context;

using convex_plane_decomposition_msgs::PlanarRegion;
using convex_plane_decomposition_msgs::PlanarTerrain;
using convex_plane_decomposition_msgs::Point2d;

namespace dairlib::geometry {

ConvexFootholdRosReceiver::ConvexFootholdRosReceiver() {
  PlanarTerrain terrain_msg;
  DeclareAbstractInputPort(
      "PlanarTerrainRosMsg", drake::Value<PlanarTerrain>(terrain_msg));
  DeclareAbstractOutputPort(
      "foolholds", &ConvexFootholdRosReceiver::CopyTerrain);
}

void ConvexFootholdRosReceiver::CopyTerrain(
    const drake::systems::Context<double> &context,
    ConvexFootholdSet *footholds) const {
  footholds->clear();
  const auto &planes =
      EvalAbstractInput(context, 0)->get_value<PlanarTerrain>();
  std::vector<ConvexFoothold> footholds_processed = DecomposeTerrain(planes);
  for (const auto &f : footholds_processed) {
    footholds->append(f);
  }
}
}