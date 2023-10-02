// STL includes
#include <iostream>
#include <chrono>

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

  foothold_state_idx_ = DeclareAbstractState(drake::Value<ConvexFootholdSet>());
  debug_state_idx_ = DeclareAbstractState(drake::Value<lcmt_convex_decomposition_debug>());

  DeclareForcedUnrestrictedUpdateEvent(
      &ConvexFootholdRosReceiver::UnrestrictedUpdate);
}

drake::systems::EventStatus ConvexFootholdRosReceiver::UnrestrictedUpdate(
    const drake::systems::Context<double>& context,
    drake::systems::State<double>* state) const {

  const auto &planes = EvalAbstractInput(context, 0)->get_value<PlanarTerrain>();
  auto start = std::chrono::high_resolution_clock::now();
  std::vector<ConvexFoothold> footholds_processed = DecomposeTerrain(
      planes, convexity_threshold_);
  auto stop = std::chrono::high_resolution_clock::now();

  state->get_mutable_abstract_state<ConvexFootholdSet>(foothold_state_idx_) =
      ConvexFootholdSet(footholds_processed);

  auto& dbg =
      state->get_mutable_abstract_state<lcmt_convex_decomposition_debug>(
          debug_state_idx_);

  dbg.utime = static_cast<int64_t>(1e6 * context.get_time());
  dbg.n_poly_in = planes.planarRegions.size();
  dbg.n_poly_out = footholds_processed.size();
  dbg.decomposition_time_us =
      std::chrono::duration_cast<std::chrono::microseconds>(stop - start).count();
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
  *footholds = context.get_abstract_state<ConvexFootholdSet>(
      foothold_state_idx_
  );
}
}