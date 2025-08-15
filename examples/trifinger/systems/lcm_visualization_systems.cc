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

LcmDensetactDrawer::LcmDensetactDrawer(
    const std::shared_ptr<drake::geometry::Meshcat>& meshcat)
    : meshcat_(meshcat) {
  densetact_input_port_ =
      this->DeclareAbstractInputPort("lcmt_densetact_data",
                                     drake::Value<dairlib::lcmt_densetact_measurement_data>{})
          .get_index();
  last_update_time_index_ = this->DeclareDiscreteState(1);
  DeclarePerStepDiscreteUpdateEvent(&LcmDensetactDrawer::DrawDensetact);
}

drake::systems::EventStatus LcmDensetactDrawer::DrawDensetact(
    const drake::systems::Context<double>& context,
    drake::systems::DiscreteValues<double>* discrete_state) const {
  if (discrete_state->get_value(last_update_time_index_)[0] <= 0.) {
    // First Run
    for(std::string path : densetact_target_paths_) {
      meshcat_->SetProperty(path, "visible", false, 0);
      meshcat_->SetObject(path, sphere_for_densetact_, {0.9, 0.1, 0.9, 0.5});
    }
    discrete_state->get_mutable_value(last_update_time_index_)[0] = 0.001; // 1ms
    return drake::systems::EventStatus::Succeeded();
  }
  const auto& densetact_data = this->EvalInputValue<dairlib::lcmt_densetact_measurement_data>(
                                  context, densetact_input_port_);
  if(densetact_data->numSensors < 1) {
    // no need to update if no data is received
    return drake::systems::EventStatus::Succeeded();
  }
  if(densetact_data->sensorData[0].timestamp / 1e6 <= discrete_state->get_value(last_update_time_index_)[0]) {
    // no need to update if data is old
    return drake::systems::EventStatus::Succeeded();
  }
  discrete_state->get_mutable_value(last_update_time_index_)[0] = densetact_data->sensorData[0].timestamp / 1e6;

  Eigen::Vector3d scaledZ;
  scaledZ << 0., 0., 0.03;
  Eigen::Matrix3d contactRot;
  Eigen::MatrixXd vertices(3, 2);
  vertices.col(0) << 0.0, 0.0, 0.0;
  vertices.col(1) = scaledZ;

  for(int8_t sensorid = 0; sensorid < densetact_data->numSensors; sensorid++) {
    if(sensorid >= (int8_t)densetact_target_paths_.size()) break;
    auto sensorData = densetact_data->sensorData[sensorid];
    std::string path = densetact_target_paths_[sensorid];
    meshcat_->SetProperty(path, "visible", static_cast<bool>(sensorData.inContact), 0);

    // Find normal vector
    for(int rowid = 0; rowid < 3; rowid++) {
      contactRot.row(rowid) << sensorData.contactFrame[rowid][0], sensorData.contactFrame[rowid][1], sensorData.contactFrame[rowid][2];
    }
    
    vertices.col(1) = contactRot * scaledZ;
    meshcat_->SetLine(path + "/normal", vertices, 5.0, {0.9, 0.1, 0.9, 1.0});
  }

  return drake::systems::EventStatus::Succeeded();
}
}  // namespace dairlib::systems