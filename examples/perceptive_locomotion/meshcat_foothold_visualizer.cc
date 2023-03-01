#include <utility>

#include "dairlib/lcmt_mpc_debug.hpp"
#include "meshcat_foothold_visualizer.h"
#include "geometry/convex_foothold_set.h"


namespace dairlib::perceptive_locomotion {
using geometry::ConvexFoothold;
using geometry::ConvexFootholdSet;

MeshcatFootholdVisualizer::MeshcatFootholdVisualizer(
    std::shared_ptr<drake::geometry::Meshcat> meshcat) :
    meshcat_(std::move(meshcat)) {

  mpc_debug_input_port_ = DeclareAbstractInputPort(
      "mpc_debug", drake::Value<lcmt_mpc_debug>()
    ).get_index();
  n_footholds_idx_ = DeclareDiscreteState(1);

  DeclarePerStepUnrestrictedUpdateEvent(
      &MeshcatFootholdVisualizer::UnrestrictedUpdate);
}

drake::systems::EventStatus MeshcatFootholdVisualizer::UnrestrictedUpdate(
    const drake::systems::Context<double> &context,
    drake::systems::State<double> *state) const {

  const auto& mpc_debug = EvalAbstractInput(
      context, mpc_debug_input_port_)->get_value<lcmt_mpc_debug>();
  auto foothold_set = ConvexFootholdSet::CopyFromLcm(mpc_debug.footholds);

  for (int i = 0; i < foothold_set.size(); i++) {
    auto foothold = foothold_set.footholds().at(i);
    const auto [verts, faces] = foothold.GetSurfaceMesh();
    meshcat_->SetTriangleMesh(make_path(i), verts, faces);
  }

  int n_prev = state->get_discrete_state(n_footholds_idx_).get_value()(0);
  for (int i = foothold_set.size(); i < n_prev; i++) {
    meshcat_->Delete(make_path(i));
  }
  return drake::systems::EventStatus::Succeeded();
}

}