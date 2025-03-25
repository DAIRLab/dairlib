#include "id_mpc_walking_debug_visualizer.h"
#include "dairlib/lcmt_id_mpc_walking_debug.hpp"

namespace dairlib::systems::controllers::id_mpc {

IDMPCWalkingDebugVisualizer::IDMPCWalkingDebugVisualizer(
    std::shared_ptr<drake::geometry::Meshcat> meshcat) : meshcat_(meshcat) {
  DeclareAbstractInputPort(
      "lcmt_id_walking_debug", drake::Value<lcmt_id_mpc_walking_debug>());
  DeclarePeriodicPublishEvent(1.0 / 100.0, 0.0,
                              &IDMPCWalkingDebugVisualizer::UpdateMeshcat);
}
namespace {

void DrawFootsteps(
    drake::geometry::Meshcat* meshcat,
    const std::vector<std::vector<double>> &footstep_sol) {
  DRAKE_DEMAND(meshcat != nullptr);
  std::vector<drake::geometry::Rgba> rgb = {
      drake::geometry::Rgba(1, 0, 0, 0.8),
      drake::geometry::Rgba(0, 0, 1, 0.8)
  };

  for (size_t n = 0; n < footstep_sol.size(); n++) {
    Eigen::Matrix4d X = Eigen::Matrix4d::Identity();
    std::string path = "footstep_sol_" + std::to_string(n);
    X.block<3, 1>(0, 3) = Eigen::Vector3d::Map(footstep_sol.at(n).data());
    const auto sphere = drake::geometry::Sphere(.025);
    meshcat->SetObject(path, sphere, rgb.at(n % 2));
    meshcat->SetTransform(path, X);
  }
}
}
void IDMPCWalkingDebugVisualizer::UpdateMeshcat(
    const drake::systems::Context<double> &context) const {
  const auto& debug_msg =
      get_input_port().Eval<lcmt_id_mpc_walking_debug>(context);
  DrawFootsteps(meshcat_.get(), debug_msg.footsteps);
}

}