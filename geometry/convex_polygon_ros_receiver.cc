// STL includes
#include <iostream>
#include <chrono>

// dair includes
#include "polygon_utils.h"
#include "convex_polygon_ros_receiver.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;
using Eigen::RowVector3d;
using drake::systems::Context;

using convex_plane_decomposition_msgs::PlanarRegion;
using convex_plane_decomposition_msgs::PlanarTerrain;
using convex_plane_decomposition_msgs::Point2d;

namespace dairlib::geometry {

ConvexPolygonRosReceiver::ConvexPolygonRosReceiver(double convex_threshold) :
  convexity_threshold_(convex_threshold){
  PlanarTerrain terrain_msg;
  DeclareAbstractInputPort(
      "PlanarTerrainRosMsg", drake::Value<PlanarTerrain>(terrain_msg));
  foothold_output_port_ = DeclareAbstractOutputPort(
      "foolholds", &ConvexPolygonRosReceiver::CopyTerrain).get_index();
  debug_output_port_ = DeclareAbstractOutputPort(
      "profiling_info", &ConvexPolygonRosReceiver::CopyDebug).get_index();

  foothold_state_idx_ = DeclareAbstractState(drake::Value<ConvexPolygonSet>());
  debug_state_idx_ = DeclareAbstractState(drake::Value<lcmt_convex_decomposition_debug>());

  DeclareForcedUnrestrictedUpdateEvent(
      &ConvexPolygonRosReceiver::UnrestrictedUpdate);
}

drake::systems::EventStatus ConvexPolygonRosReceiver::UnrestrictedUpdate(
    const drake::systems::Context<double>& context,
    drake::systems::State<double>* state) const {

  const auto &planes = EvalAbstractInput(context, 0)->get_value<PlanarTerrain>();
  auto start = std::chrono::high_resolution_clock::now();
  std::vector<ConvexPolygon> footholds_processed = DecomposeTerrain(
      planes, convexity_threshold_);
  auto stop = std::chrono::high_resolution_clock::now();

  state->get_mutable_abstract_state<ConvexPolygonSet>(foothold_state_idx_) =
      ConvexPolygonSet(footholds_processed);

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

void ConvexPolygonRosReceiver::CopyDebug(
    const drake::systems::Context<double>& context,
    lcmt_convex_decomposition_debug *debug) const {
  *debug = context.get_abstract_state<lcmt_convex_decomposition_debug>(
      debug_state_idx_
  );
}
void ConvexPolygonRosReceiver::CopyTerrain(
    const drake::systems::Context<double> &context,
    ConvexPolygonSet *footholds) const {
  *footholds = context.get_abstract_state<ConvexPolygonSet>(
      foothold_state_idx_
  );
}
}