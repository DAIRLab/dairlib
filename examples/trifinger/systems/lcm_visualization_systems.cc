#include "lcm_visualization_systems.h"

namespace dairlib::systems {
LcmCubeTargetDrawer::LcmCubeTargetDrawer(
    const std::shared_ptr<drake::geometry::Meshcat>& meshcat)
    : meshcat_(meshcat) {
  cube_target_input_port_ =
      this->DeclareAbstractInputPort("lcmt_cube_target",
                                     drake::Value<dairlib::lcmt_object_state>{})
          .get_index();
  last_update_time_index_ = this->DeclareDiscreteState(1);
  meshcat_->SetProperty(cube_target_path_, "visible", true, 0);
  meshcat_->SetObject(cube_target_path_, box_for_cube_target_, {1, 0, 0, 0.3});
  DeclarePerStepDiscreteUpdateEvent(&LcmCubeTargetDrawer::DrawCubeTarget);
}

drake::systems::EventStatus LcmCubeTargetDrawer::DrawCubeTarget(
    const drake::systems::Context<double>& context,
    drake::systems::DiscreteValues<double>* discrete_state) const {
  if (static_cast<double>(this->EvalInputValue<dairlib::lcmt_object_state>(
                                  context, cube_target_input_port_)
                              ->utime) < 1e-3) {
    return drake::systems::EventStatus::Succeeded();
  }
  if (discrete_state->get_value(last_update_time_index_)[0] >=
      context.get_time()) {
    // no need to update if simulation has not advanced
    return drake::systems::EventStatus::Succeeded();
  }
  discrete_state->get_mutable_value(last_update_time_index_)[0] =
      context.get_time();
  const auto& cube_target = this->EvalInputValue<dairlib::lcmt_object_state>(
      context, cube_target_input_port_);
  drake::log()->info(cube_target->position[0]);
  meshcat_->SetTransform(
      cube_target_path_,
      drake::math::RigidTransformd(
          Eigen::Quaterniond(cube_target->position[0], cube_target->position[1],
                             cube_target->position[2],
                             cube_target->position[3]),
          Eigen::Vector3d{cube_target->position[4], cube_target->position[5],
                          cube_target->position[6]}),
      context.get_time());
  return drake::systems::EventStatus::Succeeded();
}
}  // namespace dairlib::systems