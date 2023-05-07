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

ConvexFootholdRosReceiver::ConvexFootholdRosReceiver(double convex_threshold) :
  convexity_threshold_(convex_threshold){
  PlanarTerrain terrain_msg;
  DeclareAbstractInputPort(
      "PlanarTerrainRosMsg", drake::Value<PlanarTerrain>(terrain_msg));
  foothold_output_port_ = DeclareAbstractOutputPort(
      "foolholds", &ConvexFootholdRosReceiver::CopyTerrain).get_index();
  debug_output_port_ = DeclareAbstractOutputPort(
      "profiling_info", &ConvexFootholdRosReceiver::CopyDebug).get_index();
}

drake::systems::EventStatus UnrestrictedUpdate(
    const drake::systems::Context<double>& context,
    drake::systems::State<double>* state) {

  return drake::systems::EventStatus::Succeeded();
}

void ConvexFootholdRosReceiver::CopyDebug(
    const drake::systems::Context<double>& context,
    lcmt_convex_decomposition_debug *debug) const {
  *debug = context.get_abstract_state<lcmt_convex_decomposition_debug>(
      debug_state_idx_
      );
}
void ConvexFootholdRosReceiver::CopyTerrain(
    const drake::systems::Context<double> &context,
    ConvexFootholdSet *footholds) const {
  footholds->clear();
  const auto &planes =
      EvalAbstractInput(context, 0)->get_value<PlanarTerrain>();
  std::vector<ConvexFoothold> footholds_processed =
      DecomposeTerrain(planes, convexity_threshold_);
  for (const auto &f : footholds_processed) {
    footholds->append(f);
  }
}
}