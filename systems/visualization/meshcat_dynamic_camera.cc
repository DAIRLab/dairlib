#include "systems/visualization/meshcat_dynamic_camera.h"

#include <iostream>

#include "systems/framework/output_vector.h"

namespace dairlib {
using systems::OutputVector;

MeshcatDynamicCamera::MeshcatDynamicCamera(
    const drake::multibody::MultibodyPlant<double>& plant,
    drake::systems::Context<double>* plant_context,
    const std::shared_ptr<drake::geometry::Meshcat>& meshcat,
    const drake::multibody::RigidBodyFrame<double>& body_frame_to_track)
    : plant_(plant),
      plant_context_(plant_context),
      body_frame_to_track_(body_frame_to_track),
      meshcat_(meshcat) {
  state_port_ = this->DeclareVectorInputPort(
          "x, u, t", OutputVector<double>(plant.num_positions(),
                                          plant.num_velocities(),
                                          plant.num_actuators()))
      .get_index();
  DeclarePerStepDiscreteUpdateEvent(&MeshcatDynamicCamera::UpdateMeshcat);
}

drake::systems::EventStatus MeshcatDynamicCamera::UpdateMeshcat(
    const drake::systems::Context<double>& context,
    drake::systems::DiscreteValues<double>* discrete_state) const {
  const OutputVector<double>* robot_output =
      (OutputVector<double>*)this->EvalVectorInput(context, state_port_);
  plant_.SetPositions(plant_context_, robot_output->GetPositions());
  Eigen::VectorXd q = robot_output->GetPositions();
  Eigen::VectorXd body_pos_in_world = Eigen::VectorXd::Zero(3);
  plant_.CalcPointsPositions(*plant_context_, body_frame_to_track_,
                             Eigen::VectorXd::Zero(3), plant_.world_frame(),
                             &body_pos_in_world);
  body_pos_in_world[2] = 0.8;
  Eigen::VectorXd camera_pos_offset = Eigen::VectorXd::Zero(3);
  camera_pos_offset << 0.1, 3.0, 0.5;
  meshcat_->SetCameraPose(body_pos_in_world + camera_pos_offset, body_pos_in_world);
  return drake::systems::EventStatus::Succeeded();
}

}  // namespace dairlib